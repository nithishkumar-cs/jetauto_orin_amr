#include "cpu_preprocessor.hpp"

#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include "cpu_reference.hpp"

namespace perception_detector
{
namespace
{

void check_cuda(cudaError_t result, const std::string & operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(operation + ": " + cudaGetErrorString(result));
  }
}

}  // namespace

const char * YoloCpuPreprocessor::name() const
{
  return "cpu";
}

LetterboxTransform YoloCpuPreprocessor::preprocess(
  const ImageView & image, std::size_t input_width, std::size_t input_height,
  const YoloPreprocessDestination & destination)
{
  if (destination.data == nullptr) {
    throw std::invalid_argument("CPU preprocessing destination cannot be null");
  }

  auto result = preprocess_yolo_cpu_reference(image, input_width, input_height);
  const auto element_size =
    destination.type == YoloTensorType::FLOAT32 ? sizeof(float) : sizeof(__half);
  const auto required_bytes = result.values.size() * element_size;
  if (destination.size_bytes < required_bytes) {
    throw std::invalid_argument("CPU preprocessing destination buffer is too small");
  }

  if (destination.type == YoloTensorType::FLOAT32) {
    float_values_ = std::move(result.values);
    check_cuda(
      cudaMemcpyAsync(
        destination.data, float_values_.data(), required_bytes, cudaMemcpyHostToDevice,
        destination.stream),
      "cudaMemcpyAsync CPU-preprocessed input");
  } else {
    half_values_.resize(result.values.size());
    std::transform(
      result.values.begin(), result.values.end(), half_values_.begin(),
      [](float value) { return __float2half(value); });
    check_cuda(
      cudaMemcpyAsync(
        destination.data, half_values_.data(), required_bytes, cudaMemcpyHostToDevice,
        destination.stream),
      "cudaMemcpyAsync CPU-preprocessed FP16 input");
  }
  return result.transform;
}

}  // namespace perception_detector
