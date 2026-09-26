#ifndef PERCEPTION_DETECTOR__YOLO_POSTPROCESSING_HPP_
#define PERCEPTION_DETECTOR__YOLO_POSTPROCESSING_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "perception_detector/detector_backend.hpp"
#include "perception_detector/yolo_tensorrt_backend.hpp"

namespace perception_detector
{

struct LetterboxTransform
{
  std::size_t input_width{0U};
  std::size_t input_height{0U};
  double scale_x{1.0};
  double scale_y{1.0};
  double pad_left{0.0};
  double pad_top{0.0};
};

std::vector<Detection> postprocess_yolo(
  const std::vector<float> & output, const std::vector<std::int64_t> & output_shape,
  const LetterboxTransform & transform, std::size_t image_width, std::size_t image_height,
  const std::vector<std::string> & labels, const YoloPostprocessConfig & config);

std::vector<std::string> load_labels(const std::string & path);
YoloOutputFormat yolo_output_format_from_string(const std::string & value);

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__YOLO_POSTPROCESSING_HPP_
