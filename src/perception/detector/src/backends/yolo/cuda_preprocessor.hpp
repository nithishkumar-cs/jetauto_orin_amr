#ifndef PERCEPTION_DETECTOR__YOLO_CUDA_PREPROCESSOR_HPP_
#define PERCEPTION_DETECTOR__YOLO_CUDA_PREPROCESSOR_HPP_

#include <cstddef>
#include <cstdint>

#include "preprocessor.hpp"

namespace perception_detector
{

// Owns and reuses the packed device buffer for the source RGB/BGR image. The
// fused CUDA kernel writes normalized planar RGB directly into TensorRT input.
class YoloCudaPreprocessor final : public YoloPreprocessor
{
public:
  YoloCudaPreprocessor() = default;
  ~YoloCudaPreprocessor();

  YoloCudaPreprocessor(const YoloCudaPreprocessor &) = delete;
  YoloCudaPreprocessor & operator=(const YoloCudaPreprocessor &) = delete;

  const char * name() const override;
  LetterboxTransform preprocess(
    const ImageView & image, std::size_t input_width, std::size_t input_height,
    const YoloPreprocessDestination & destination) override;

private:
  void ensure_source_capacity(std::size_t required_bytes);

  std::uint8_t * source_device_{nullptr};
  std::size_t source_capacity_bytes_{0U};
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__YOLO_CUDA_PREPROCESSOR_HPP_
