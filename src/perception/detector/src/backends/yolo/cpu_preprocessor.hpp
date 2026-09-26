#ifndef PERCEPTION_DETECTOR__YOLO_CPU_PREPROCESSOR_HPP_
#define PERCEPTION_DETECTOR__YOLO_CPU_PREPROCESSOR_HPP_

#include <cuda_fp16.h>

#include <vector>

#include "preprocessor.hpp"

namespace perception_detector
{

class YoloCpuPreprocessor final : public YoloPreprocessor
{
public:
  const char * name() const override;
  LetterboxTransform preprocess(
    const ImageView & image, std::size_t input_width, std::size_t input_height,
    const YoloPreprocessDestination & destination) override;

private:
  std::vector<float> float_values_;
  std::vector<__half> half_values_;
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__YOLO_CPU_PREPROCESSOR_HPP_
