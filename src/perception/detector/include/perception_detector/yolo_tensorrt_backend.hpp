#ifndef PERCEPTION_DETECTOR__YOLO_TENSORRT_BACKEND_HPP_
#define PERCEPTION_DETECTOR__YOLO_TENSORRT_BACKEND_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "perception_detector/detector_backend.hpp"

namespace perception_detector
{

enum class YoloPreprocessingBackend {
  CPU,
  CUDA,
};

enum class YoloOutputFormat {
  // One tensor shaped [1, 4 + classes, predictions] or its transpose.
  RAW_XYWH,
  // Native end-to-end output shaped [1, predictions, 6]: x1,y1,x2,y2,score,class.
  END_TO_END_XYXY,
};

struct YoloPostprocessConfig
{
  double confidence_threshold{0.35};
  double nms_iou_threshold{0.45};
  std::size_t max_detections{100U};
  YoloOutputFormat output_format{YoloOutputFormat::RAW_XYWH};
};

struct YoloTensorRtConfig
{
  std::string engine_path;
  std::string labels_path;
  std::string input_tensor_name;
  std::string output_tensor_name;
  std::size_t dynamic_input_width{640U};
  std::size_t dynamic_input_height{640U};
  YoloPreprocessingBackend preprocessing_backend{YoloPreprocessingBackend::CUDA};
  YoloPostprocessConfig postprocess;
};

class YoloTensorRtBackend final : public DetectorBackend
{
public:
  explicit YoloTensorRtBackend(YoloTensorRtConfig config);
  ~YoloTensorRtBackend() override;

  YoloTensorRtBackend(const YoloTensorRtBackend &) = delete;
  YoloTensorRtBackend & operator=(const YoloTensorRtBackend &) = delete;
  YoloTensorRtBackend(YoloTensorRtBackend &&) noexcept;
  YoloTensorRtBackend & operator=(YoloTensorRtBackend &&) noexcept;

  std::string name() const override;
  std::vector<Detection> detect(const ImageView & image) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__YOLO_TENSORRT_BACKEND_HPP_
