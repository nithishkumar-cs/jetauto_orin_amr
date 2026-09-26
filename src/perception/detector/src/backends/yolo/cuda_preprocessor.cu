#include <cuda_fp16.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "cuda_preprocessor.hpp"

namespace perception_detector
{
namespace
{

constexpr float kLetterboxValue = 114.0F / 255.0F;
constexpr unsigned int kBlockSize = 16U;

void check_cuda(cudaError_t result, const std::string & operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(operation + ": " + cudaGetErrorString(result));
  }
}

template <typename Output>
__device__ Output convert_output(float value);

template <>
__device__ float convert_output<float>(float value)
{
  return value;
}

template <>
__device__ __half convert_output<__half>(float value)
{
  return __float2half_rn(value);
}

template <typename Output>
__global__ void preprocess_kernel(
  const std::uint8_t * source, int source_width, int source_height, bool source_is_bgr,
  Output * destination, int input_width, int input_height, int resized_width, int resized_height,
  int pad_left, int pad_top, float scale_x, float scale_y)
{
  const auto output_x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const auto output_y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (output_x >= input_width || output_y >= input_height) {
    return;
  }

  const auto plane_size = input_width * input_height;
  const auto output_index = output_y * input_width + output_x;
  const auto inside_image = output_x >= pad_left && output_x < pad_left + resized_width &&
                            output_y >= pad_top && output_y < pad_top + resized_height;
  if (!inside_image) {
    const auto padding = convert_output<Output>(kLetterboxValue);
    destination[output_index] = padding;
    destination[plane_size + output_index] = padding;
    destination[2 * plane_size + output_index] = padding;
    return;
  }

  const auto resized_x = output_x - pad_left;
  const auto resized_y = output_y - pad_top;
  const auto source_x = fminf(
    fmaxf((static_cast<float>(resized_x) + 0.5F) / scale_x - 0.5F, 0.0F),
    static_cast<float>(source_width - 1));
  const auto source_y = fminf(
    fmaxf((static_cast<float>(resized_y) + 0.5F) / scale_y - 0.5F, 0.0F),
    static_cast<float>(source_height - 1));
  const auto left = static_cast<int>(floorf(source_x));
  const auto right = min(left + 1, source_width - 1);
  const auto top = static_cast<int>(floorf(source_y));
  const auto bottom = min(top + 1, source_height - 1);
  const auto horizontal_weight = source_x - static_cast<float>(left);
  const auto vertical_weight = source_y - static_cast<float>(top);

  for (int output_channel = 0; output_channel < 3; ++output_channel) {
    const auto source_channel = source_is_bgr ? 2 - output_channel : output_channel;
    const auto top_left =
      static_cast<float>(source[(top * source_width + left) * 3 + source_channel]);
    const auto top_right =
      static_cast<float>(source[(top * source_width + right) * 3 + source_channel]);
    const auto bottom_left =
      static_cast<float>(source[(bottom * source_width + left) * 3 + source_channel]);
    const auto bottom_right =
      static_cast<float>(source[(bottom * source_width + right) * 3 + source_channel]);
    const auto top_value = top_left * (1.0F - horizontal_weight) + top_right * horizontal_weight;
    const auto bottom_value =
      bottom_left * (1.0F - horizontal_weight) + bottom_right * horizontal_weight;
    const auto value =
      (top_value * (1.0F - vertical_weight) + bottom_value * vertical_weight) / 255.0F;
    destination[output_channel * plane_size + output_index] = convert_output<Output>(value);
  }
}

std::size_t checked_tensor_elements(std::size_t input_width, std::size_t input_height)
{
  if (input_width == 0U || input_height == 0U) {
    throw std::invalid_argument("CUDA preprocessing input dimensions must be positive");
  }
  if (
    input_width > std::numeric_limits<std::size_t>::max() / input_height ||
    input_width * input_height > std::numeric_limits<std::size_t>::max() / 3U) {
    throw std::overflow_error("CUDA preprocessing input dimensions are too large");
  }
  return 3U * input_width * input_height;
}

}  // namespace

YoloCudaPreprocessor::~YoloCudaPreprocessor()
{
  if (source_device_ != nullptr) {
    cudaFree(source_device_);
  }
}

LetterboxTransform YoloCudaPreprocessor::preprocess(
  const ImageView & image, std::size_t input_width, std::size_t input_height,
  const YoloPreprocessDestination & destination)
{
  if (destination.data == nullptr) {
    throw std::invalid_argument("CUDA preprocessing destination cannot be null");
  }
  if (
    image.width() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
    image.height() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
    input_width > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
    input_height > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::overflow_error("CUDA preprocessing dimensions exceed kernel integer range");
  }

  const auto tensor_elements = checked_tensor_elements(input_width, input_height);
  const auto element_size =
    destination.type == YoloTensorType::FLOAT32 ? sizeof(float) : sizeof(__half);
  if (tensor_elements > std::numeric_limits<std::size_t>::max() / element_size) {
    throw std::overflow_error("CUDA preprocessing output size is too large");
  }
  const auto required_destination_bytes = tensor_elements * element_size;
  if (destination.size_bytes < required_destination_bytes) {
    throw std::invalid_argument("CUDA preprocessing destination buffer is too small");
  }

  constexpr std::size_t channels = 3U;
  const auto source_row_bytes = image.width() * channels;
  if (source_row_bytes > std::numeric_limits<std::size_t>::max() / image.height()) {
    throw std::overflow_error("CUDA preprocessing source image is too large");
  }
  ensure_source_capacity(source_row_bytes * image.height());
  check_cuda(
    cudaMemcpy2DAsync(
      source_device_, source_row_bytes, image.data(), image.row_step_bytes(), source_row_bytes,
      image.height(), cudaMemcpyHostToDevice, destination.stream),
    "cudaMemcpy2DAsync source image");

  const auto scale = std::min(
    static_cast<double>(input_width) / static_cast<double>(image.width()),
    static_cast<double>(input_height) / static_cast<double>(image.height()));
  const auto resized_width = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::llround(static_cast<double>(image.width()) * scale)));
  const auto resized_height = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::llround(static_cast<double>(image.height()) * scale)));
  if (resized_width > input_width || resized_height > input_height) {
    throw std::runtime_error("CUDA preprocessing computed an invalid letterbox size");
  }
  const auto pad_left = (input_width - resized_width) / 2U;
  const auto pad_top = (input_height - resized_height) / 2U;
  const LetterboxTransform transform{
    input_width,
    input_height,
    static_cast<double>(resized_width) / static_cast<double>(image.width()),
    static_cast<double>(resized_height) / static_cast<double>(image.height()),
    static_cast<double>(pad_left),
    static_cast<double>(pad_top)};

  const dim3 block{kBlockSize, kBlockSize};
  const dim3 grid{
    static_cast<unsigned int>((input_width + kBlockSize - 1U) / kBlockSize),
    static_cast<unsigned int>((input_height + kBlockSize - 1U) / kBlockSize)};
  const auto source_is_bgr = image.encoding() == PixelEncoding::BGR8;
  if (destination.type == YoloTensorType::FLOAT32) {
    preprocess_kernel<<<grid, block, 0, destination.stream>>>(
      source_device_, static_cast<int>(image.width()), static_cast<int>(image.height()),
      source_is_bgr, static_cast<float *>(destination.data), static_cast<int>(input_width),
      static_cast<int>(input_height), static_cast<int>(resized_width),
      static_cast<int>(resized_height), static_cast<int>(pad_left), static_cast<int>(pad_top),
      static_cast<float>(transform.scale_x), static_cast<float>(transform.scale_y));
  } else {
    preprocess_kernel<<<grid, block, 0, destination.stream>>>(
      source_device_, static_cast<int>(image.width()), static_cast<int>(image.height()),
      source_is_bgr, static_cast<__half *>(destination.data), static_cast<int>(input_width),
      static_cast<int>(input_height), static_cast<int>(resized_width),
      static_cast<int>(resized_height), static_cast<int>(pad_left), static_cast<int>(pad_top),
      static_cast<float>(transform.scale_x), static_cast<float>(transform.scale_y));
  }
  check_cuda(cudaGetLastError(), "CUDA YOLO preprocessing kernel launch");
  return transform;
}

const char * YoloCudaPreprocessor::name() const
{
  return "cuda";
}

void YoloCudaPreprocessor::ensure_source_capacity(std::size_t required_bytes)
{
  if (required_bytes <= source_capacity_bytes_) {
    return;
  }

  std::uint8_t * replacement = nullptr;
  check_cuda(
    cudaMalloc(reinterpret_cast<void **>(&replacement), required_bytes), "cudaMalloc source image");
  if (source_device_ != nullptr) {
    check_cuda(cudaFree(source_device_), "cudaFree previous source image");
  }
  source_device_ = replacement;
  source_capacity_bytes_ = required_bytes;
}

}  // namespace perception_detector
