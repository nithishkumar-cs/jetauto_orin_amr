#ifndef PERCEPTION_DETECTOR__YOLO_CPU_REFERENCE_HPP_
#define PERCEPTION_DETECTOR__YOLO_CPU_REFERENCE_HPP_

#include <cstddef>
#include <vector>

#include "perception_detector/detector_backend.hpp"
#include "postprocessing.hpp"

namespace perception_detector
{

struct CpuPreprocessedImage
{
  // RGB, planar CHW, float32, normalized to [0, 1].
  std::vector<float> values;
  LetterboxTransform transform;
};

CpuPreprocessedImage preprocess_yolo_cpu_reference(
  const ImageView & image, std::size_t input_width, std::size_t input_height);

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__YOLO_CPU_REFERENCE_HPP_
