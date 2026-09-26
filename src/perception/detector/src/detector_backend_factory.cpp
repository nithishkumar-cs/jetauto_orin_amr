#include "perception_detector/detector_backend_factory.hpp"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "perception_detector/yolo_tensorrt_backend.hpp"
#include "rclcpp/rclcpp.hpp"
#include "backends/yolo/postprocessing.hpp"

namespace perception_detector
{
namespace
{

std::unique_ptr<DetectorBackend> create_yolo_tensorrt_backend(rclcpp::Node & node)
{
  const auto model_architecture =
    node.declare_parameter<std::string>("model_architecture", "yolo26");
  const auto engine_path = node.declare_parameter<std::string>("engine_path", "");
  const auto labels_path = node.declare_parameter<std::string>("labels_path", "");
  const auto input_tensor_name = node.declare_parameter<std::string>("input_tensor_name", "");
  const auto output_tensor_name = node.declare_parameter<std::string>("output_tensor_name", "");
  const auto dynamic_input_width = node.declare_parameter<int>("dynamic_input_width", 640);
  const auto dynamic_input_height = node.declare_parameter<int>("dynamic_input_height", 640);
  const auto preprocessing_backend =
    node.declare_parameter<std::string>("preprocessing_backend", "cuda");
  const auto confidence_threshold = node.declare_parameter<double>("confidence_threshold", 0.35);
  const auto nms_iou_threshold = node.declare_parameter<double>("nms_iou_threshold", 0.45);
  const auto max_detections = node.declare_parameter<int>("max_detections", 100);
  const auto output_format =
    node.declare_parameter<std::string>("output_format", "end_to_end_xyxy");

  if (model_architecture != "yolo26") {
    throw std::invalid_argument(
      "unsupported model_architecture '" + model_architecture +
      "' for backend 'yolo_tensorrt'; this build currently provides yolo26");
  }
  if (
    dynamic_input_width <= 0 || dynamic_input_height <= 0 || max_detections <= 0 ||
    confidence_threshold < 0.0 || confidence_threshold > 1.0 || nms_iou_threshold < 0.0 ||
    nms_iou_threshold > 1.0) {
    throw std::invalid_argument("YOLO TensorRT backend numeric parameters are invalid");
  }

  YoloTensorRtConfig config;
  if (preprocessing_backend == "cpu") {
    config.preprocessing_backend = YoloPreprocessingBackend::CPU;
  } else if (preprocessing_backend == "cuda") {
    config.preprocessing_backend = YoloPreprocessingBackend::CUDA;
  } else {
    throw std::invalid_argument(
      "unsupported preprocessing_backend '" + preprocessing_backend + "'; expected cpu or cuda");
  }

  config.engine_path = engine_path;
  config.labels_path = labels_path;
  config.input_tensor_name = input_tensor_name;
  config.output_tensor_name = output_tensor_name;
  config.dynamic_input_width = static_cast<std::size_t>(dynamic_input_width);
  config.dynamic_input_height = static_cast<std::size_t>(dynamic_input_height);
  config.postprocess.confidence_threshold = confidence_threshold;
  config.postprocess.nms_iou_threshold = nms_iou_threshold;
  config.postprocess.max_detections = static_cast<std::size_t>(max_detections);
  config.postprocess.output_format = yolo_output_format_from_string(output_format);
  return std::make_unique<YoloTensorRtBackend>(std::move(config));
}

}  // namespace

std::unique_ptr<DetectorBackend> create_detector_backend(rclcpp::Node & node)
{
  const auto backend = node.declare_parameter<std::string>("backend", "yolo_tensorrt");
  if (backend == "yolo_tensorrt") {
    return create_yolo_tensorrt_backend(node);
  }
  throw std::invalid_argument(
    "unsupported detector backend '" + backend + "'; expected yolo_tensorrt");
}

}  // namespace perception_detector
