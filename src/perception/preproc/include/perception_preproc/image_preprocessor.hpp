#pragma once

#include "cuda_common/cuda_event_timer.hpp"
#include "cuda_common/cuda_stream.hpp"
#include "cuda_common/device_buffer.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace perception_preproc
{

struct ImagePreprocessConfig
{
  int network_width{640};
  int network_height{640};
  bool keep_aspect_ratio{true};
  float pad_value{114.0f};
  float scale{1.0f / 255.0f};
  float mean[3]{0.0f, 0.0f, 0.0f};
  float stddev[3]{1.0f, 1.0f, 1.0f};
};

struct HostImageView
{
  const uint8_t* data{nullptr};
  int width{0};
  int height{0};
  int step{0};
  int channels{0};
  bool input_is_bgr{true};
};

struct PreprocessStats
{
  float kernel_ms{0.0f};
  std::size_t input_bytes{0};
  std::size_t output_elements{0};
};

class GpuImagePreprocessor
{
public:
  explicit GpuImagePreprocessor(ImagePreprocessConfig config);

  const ImagePreprocessConfig& config() const noexcept { return config_; }
  const float* device_output() const noexcept { return device_output_.data(); }
  std::size_t output_elements() const noexcept { return output_elements_; }

  PreprocessStats preprocess_to_host(const HostImageView& image, std::vector<float>& output);

private:
  ImagePreprocessConfig config_;
  cuda_common::CudaStream stream_;
  cuda_common::CudaEventTimer timer_;
  cuda_common::DeviceBuffer<uint8_t> device_input_;
  cuda_common::DeviceBuffer<float> device_output_;
  std::size_t output_elements_{0};
};

}  // namespace perception_preproc

