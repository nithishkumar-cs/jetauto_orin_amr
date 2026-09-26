#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "backends/yolo/cpu_preprocessor.hpp"
#include "backends/yolo/cpu_reference.hpp"
#include "backends/yolo/cuda_preprocessor.hpp"
#include "backends/yolo/postprocessing.hpp"
#include "backends/yolo/preprocessor.hpp"
#include "perception_detector/detector_backend.hpp"

namespace perception_detector
{
namespace
{

class CudaBuffer
{
public:
  explicit CudaBuffer(std::size_t size_bytes)
  {
    if (cudaMalloc(&data_, size_bytes) != cudaSuccess) {
      data_ = nullptr;
    }
  }

  ~CudaBuffer()
  {
    if (data_ != nullptr) {
      cudaFree(data_);
    }
  }

  void * data() const
  {
    return data_;
  }

private:
  void * data_{nullptr};
};

bool cuda_device_available()
{
  int device_count = 0;
  return cudaGetDeviceCount(&device_count) == cudaSuccess && device_count > 0;
}

ImageView make_image_view(
  const std::vector<std::uint8_t> & data, std::size_t width, std::size_t height,
  std::size_t row_step_bytes, PixelEncoding encoding)
{
  return ImageView::create(width, height, row_step_bytes, encoding, data.data(), data.size())
    .value();
}

TEST(YoloPreprocessingBackendsTest, CudaMatchesCpuReferenceForPaddedBgrInput)
{
  if (!cuda_device_available()) {
    GTEST_SKIP() << "CUDA device is unavailable";
  }

  constexpr std::size_t source_width = 3U;
  constexpr std::size_t source_height = 2U;
  constexpr std::size_t source_step = 11U;
  constexpr std::size_t input_width = 6U;
  constexpr std::size_t input_height = 6U;
  std::vector<std::uint8_t> data(source_step * source_height, 0U);
  const std::vector<std::uint8_t> pixels{
    0U, 0U, 255U, 0U, 255U, 0U, 255U, 0U, 0U, 255U, 255U, 255U, 64U, 128U, 192U, 32U, 16U, 8U,
  };
  for (std::size_t row = 0U; row < source_height; ++row) {
    std::copy_n(
      pixels.begin() + static_cast<std::ptrdiff_t>(row * source_width * 3U), source_width * 3U,
      data.begin() + static_cast<std::ptrdiff_t>(row * source_step));
  }
  const auto image =
    make_image_view(data, source_width, source_height, source_step, PixelEncoding::BGR8);
  const auto cpu = preprocess_yolo_cpu_reference(image, input_width, input_height);

  const auto output_size_bytes = cpu.values.size() * sizeof(float);
  CudaBuffer output(output_size_bytes);
  ASSERT_NE(output.data(), nullptr);
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);
  YoloCudaPreprocessor preprocessor;
  const YoloPreprocessDestination destination{
    output.data(), output_size_bytes, YoloTensorType::FLOAT32, stream};
  const auto transform = preprocessor.preprocess(image, input_width, input_height, destination);
  std::vector<float> gpu(cpu.values.size());
  ASSERT_EQ(
    cudaMemcpyAsync(gpu.data(), output.data(), output_size_bytes, cudaMemcpyDeviceToHost, stream),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  ASSERT_EQ(cudaStreamDestroy(stream), cudaSuccess);

  EXPECT_DOUBLE_EQ(transform.scale_x, cpu.transform.scale_x);
  EXPECT_DOUBLE_EQ(transform.scale_y, cpu.transform.scale_y);
  EXPECT_DOUBLE_EQ(transform.pad_left, cpu.transform.pad_left);
  EXPECT_DOUBLE_EQ(transform.pad_top, cpu.transform.pad_top);
  ASSERT_EQ(gpu.size(), cpu.values.size());
  for (std::size_t index = 0U; index < gpu.size(); ++index) {
    EXPECT_NEAR(gpu[index], cpu.values[index], 1e-5F) << "tensor index " << index;
  }
}

TEST(YoloPreprocessingBackendsTest, CudaWritesFloat16TensorDirectly)
{
  if (!cuda_device_available()) {
    GTEST_SKIP() << "CUDA device is unavailable";
  }

  const std::vector<std::uint8_t> data{255U, 0U, 0U};
  const auto image = make_image_view(data, 1U, 1U, 3U, PixelEncoding::RGB8);
  constexpr std::size_t tensor_elements = 3U;
  constexpr std::size_t output_size_bytes = tensor_elements * sizeof(__half);
  CudaBuffer output(output_size_bytes);
  ASSERT_NE(output.data(), nullptr);
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);
  YoloCudaPreprocessor preprocessor;
  const YoloPreprocessDestination destination{
    output.data(), output_size_bytes, YoloTensorType::FLOAT16, stream};
  preprocessor.preprocess(image, 1U, 1U, destination);
  std::vector<__half> gpu(tensor_elements);
  ASSERT_EQ(
    cudaMemcpyAsync(gpu.data(), output.data(), output_size_bytes, cudaMemcpyDeviceToHost, stream),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  ASSERT_EQ(cudaStreamDestroy(stream), cudaSuccess);

  EXPECT_FLOAT_EQ(__half2float(gpu[0]), 1.0F);
  EXPECT_FLOAT_EQ(__half2float(gpu[1]), 0.0F);
  EXPECT_FLOAT_EQ(__half2float(gpu[2]), 0.0F);
}

TEST(YoloPreprocessingBackendsTest, CpuImplementationUsesTheSameDestinationContract)
{
  if (!cuda_device_available()) {
    GTEST_SKIP() << "CUDA device is unavailable";
  }

  const std::vector<std::uint8_t> data{0U, 0U, 255U, 0U, 255U, 0U};
  const auto image = make_image_view(data, 2U, 1U, 6U, PixelEncoding::BGR8);
  const auto reference = preprocess_yolo_cpu_reference(image, 4U, 4U);
  const auto output_size_bytes = reference.values.size() * sizeof(float);
  CudaBuffer output(output_size_bytes);
  ASSERT_NE(output.data(), nullptr);
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);
  YoloCpuPreprocessor preprocessor;
  const YoloPreprocessDestination destination{
    output.data(), output_size_bytes, YoloTensorType::FLOAT32, stream};
  const auto transform = preprocessor.preprocess(image, 4U, 4U, destination);
  std::vector<float> uploaded(reference.values.size());
  ASSERT_EQ(
    cudaMemcpyAsync(
      uploaded.data(), output.data(), output_size_bytes, cudaMemcpyDeviceToHost, stream),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  ASSERT_EQ(cudaStreamDestroy(stream), cudaSuccess);

  EXPECT_STREQ(preprocessor.name(), "cpu");
  EXPECT_DOUBLE_EQ(transform.scale_x, reference.transform.scale_x);
  EXPECT_DOUBLE_EQ(transform.scale_y, reference.transform.scale_y);
  EXPECT_EQ(uploaded, reference.values);
}

}  // namespace
}  // namespace perception_detector
