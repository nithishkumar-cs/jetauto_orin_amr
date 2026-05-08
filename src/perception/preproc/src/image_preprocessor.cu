#include "perception_preproc/image_preprocessor.hpp"

#include "cuda_common/cuda_check.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace perception_preproc
{
namespace
{

__device__ int clamp_int(int value, int lo, int hi)
{
  return value < lo ? lo : (value > hi ? hi : value);
}

__device__ float read_channel(
  const uint8_t* src,
  int src_width,
  int src_height,
  int src_step,
  int src_channels,
  bool input_is_bgr,
  int x,
  int y,
  int c)
{
  x = clamp_int(x, 0, src_width - 1);
  y = clamp_int(y, 0, src_height - 1);

  const uint8_t* row = src + static_cast<std::size_t>(y) * static_cast<std::size_t>(src_step);

  if (src_channels == 1) {
    return static_cast<float>(row[x]);
  }

  int src_c = c;
  if (input_is_bgr) {
    if (c == 0) {
      src_c = 2;
    } else if (c == 2) {
      src_c = 0;
    }
  }

  return static_cast<float>(row[x * src_channels + src_c]);
}

__device__ float bilinear_sample(
  const uint8_t* src,
  int src_width,
  int src_height,
  int src_step,
  int src_channels,
  bool input_is_bgr,
  float fx,
  float fy,
  int c)
{
  const int x0 = static_cast<int>(floorf(fx));
  const int y0 = static_cast<int>(floorf(fy));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const float dx = fx - static_cast<float>(x0);
  const float dy = fy - static_cast<float>(y0);

  const float v00 = read_channel(src, src_width, src_height, src_step, src_channels, input_is_bgr, x0, y0, c);
  const float v01 = read_channel(src, src_width, src_height, src_step, src_channels, input_is_bgr, x1, y0, c);
  const float v10 = read_channel(src, src_width, src_height, src_step, src_channels, input_is_bgr, x0, y1, c);
  const float v11 = read_channel(src, src_width, src_height, src_step, src_channels, input_is_bgr, x1, y1, c);

  const float top = v00 + dx * (v01 - v00);
  const float bottom = v10 + dx * (v11 - v10);
  return top + dy * (bottom - top);
}

__global__ void preprocess_kernel(
  const uint8_t* src,
  int src_width,
  int src_height,
  int src_step,
  int src_channels,
  bool input_is_bgr,
  int dst_width,
  int dst_height,
  bool keep_aspect_ratio,
  float pad_value,
  float scale,
  float mean0,
  float mean1,
  float mean2,
  float std0,
  float std1,
  float std2,
  float* dst)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int plane_size = dst_width * dst_height;
  const int total = 3 * plane_size;

  if (idx >= total) {
    return;
  }

  const int c = idx / plane_size;
  const int rem = idx % plane_size;
  const int y = rem / dst_width;
  const int x = rem % dst_width;

  float resized_w = static_cast<float>(dst_width);
  float resized_h = static_cast<float>(dst_height);
  float pad_x = 0.0f;
  float pad_y = 0.0f;
  float scale_x = static_cast<float>(src_width) / static_cast<float>(dst_width);
  float scale_y = static_cast<float>(src_height) / static_cast<float>(dst_height);

  if (keep_aspect_ratio) {
    const float r = fminf(
      static_cast<float>(dst_width) / static_cast<float>(src_width),
      static_cast<float>(dst_height) / static_cast<float>(src_height));
    resized_w = roundf(static_cast<float>(src_width) * r);
    resized_h = roundf(static_cast<float>(src_height) * r);
    pad_x = 0.5f * (static_cast<float>(dst_width) - resized_w);
    pad_y = 0.5f * (static_cast<float>(dst_height) - resized_h);
    scale_x = static_cast<float>(src_width) / resized_w;
    scale_y = static_cast<float>(src_height) / resized_h;
  }

  const bool inside =
    !keep_aspect_ratio ||
    (static_cast<float>(x) >= pad_x &&
     static_cast<float>(x) < pad_x + resized_w &&
     static_cast<float>(y) >= pad_y &&
     static_cast<float>(y) < pad_y + resized_h);

  float value = pad_value;

  if (inside) {
    const float src_x = keep_aspect_ratio ?
      ((static_cast<float>(x) - pad_x) + 0.5f) * scale_x - 0.5f :
      (static_cast<float>(x) + 0.5f) * scale_x - 0.5f;
    const float src_y = keep_aspect_ratio ?
      ((static_cast<float>(y) - pad_y) + 0.5f) * scale_y - 0.5f :
      (static_cast<float>(y) + 0.5f) * scale_y - 0.5f;

    value = bilinear_sample(
      src,
      src_width,
      src_height,
      src_step,
      src_channels,
      input_is_bgr,
      src_x,
      src_y,
      c);
  }

  const float mean = c == 0 ? mean0 : (c == 1 ? mean1 : mean2);
  const float stdv = c == 0 ? std0 : (c == 1 ? std1 : std2);

  dst[idx] = ((value * scale) - mean) / stdv;
}

void validate(const ImagePreprocessConfig& config, const HostImageView& image)
{
  if (config.network_width <= 0 || config.network_height <= 0) {
    throw std::invalid_argument("network dimensions must be positive");
  }
  if (image.data == nullptr || image.width <= 0 || image.height <= 0 || image.step <= 0) {
    throw std::invalid_argument("invalid host image");
  }
  if (image.channels != 1 && image.channels != 3) {
    throw std::invalid_argument("only mono8, rgb8, and bgr8 inputs are supported");
  }
  for (float value : config.stddev) {
    if (value == 0.0f) {
      throw std::invalid_argument("stddev values must be non-zero");
    }
  }
}

}  // namespace

GpuImagePreprocessor::GpuImagePreprocessor(ImagePreprocessConfig config)
: config_(config)
{
  output_elements_ = static_cast<std::size_t>(3) *
    static_cast<std::size_t>(config_.network_width) *
    static_cast<std::size_t>(config_.network_height);
  device_output_.reserve(output_elements_);
}

PreprocessStats GpuImagePreprocessor::preprocess_to_host(const HostImageView& image, std::vector<float>& output)
{
  validate(config_, image);

  const auto input_bytes = static_cast<std::size_t>(image.step) * static_cast<std::size_t>(image.height);
  output_elements_ = static_cast<std::size_t>(3) *
    static_cast<std::size_t>(config_.network_width) *
    static_cast<std::size_t>(config_.network_height);

  device_input_.resize(input_bytes);
  device_output_.resize(output_elements_);

  CUDA_COMMON_CHECK(cudaMemcpyAsync(
    device_input_.data(),
    image.data,
    input_bytes,
    cudaMemcpyHostToDevice,
    stream_.get()));

  const int threads = 256;
  const int blocks = static_cast<int>((output_elements_ + threads - 1) / threads);

  timer_.start(stream_.get());
  preprocess_kernel<<<blocks, threads, 0, stream_.get()>>>(
    device_input_.data(),
    image.width,
    image.height,
    image.step,
    image.channels,
    image.input_is_bgr,
    config_.network_width,
    config_.network_height,
    config_.keep_aspect_ratio,
    config_.pad_value,
    config_.scale,
    config_.mean[0],
    config_.mean[1],
    config_.mean[2],
    config_.stddev[0],
    config_.stddev[1],
    config_.stddev[2],
    device_output_.data());
  CUDA_COMMON_CHECK(cudaGetLastError());
  timer_.stop(stream_.get());

  output.resize(output_elements_);
  CUDA_COMMON_CHECK(cudaMemcpyAsync(
    output.data(),
    device_output_.data(),
    output_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost,
    stream_.get()));

  stream_.synchronize();

  PreprocessStats stats;
  stats.kernel_ms = timer_.elapsed_ms();
  stats.input_bytes = input_bytes;
  stats.output_elements = output_elements_;
  return stats;
}

}  // namespace perception_preproc

