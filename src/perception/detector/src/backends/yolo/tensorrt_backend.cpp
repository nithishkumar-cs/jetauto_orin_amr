#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "perception_detector/yolo_tensorrt_backend.hpp"

#ifdef PERCEPTION_DETECTOR_HAS_TENSORRT

#include <algorithm>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cpu_preprocessor.hpp"
#include "cuda_preprocessor.hpp"
#include "postprocessing.hpp"
#include "preprocessor.hpp"
#include "runtime/tensorrt/session.hpp"

#endif

namespace perception_detector
{

#ifdef PERCEPTION_DETECTOR_HAS_TENSORRT
namespace
{

std::unique_ptr<YoloPreprocessor> create_yolo_preprocessor(YoloPreprocessingBackend backend)
{
  if (backend == YoloPreprocessingBackend::CPU) {
    return std::make_unique<YoloCpuPreprocessor>();
  }
  return std::make_unique<YoloCudaPreprocessor>();
}

std::string select_single_tensor(
  const std::vector<std::string> & available, const std::string & configured,
  const std::string & mode)
{
  if (available.size() != 1U) {
    throw std::runtime_error("YOLO TensorRT backend requires exactly one " + mode + " tensor");
  }

  const auto & discovered = available.front();
  if (!configured.empty() && configured != discovered) {
    throw std::runtime_error(
      "configured " + mode + " tensor '" + configured + "' does not match engine " + mode + " '" +
      discovered + "'");
  }
  return discovered;
}

std::string shape_string(const std::vector<std::int64_t> & shape)
{
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0U; index < shape.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << shape[index];
  }
  output << ']';
  return output.str();
}

YoloTensorType yolo_tensor_type(tensorrt::TensorType type)
{
  return type == tensorrt::TensorType::FLOAT32 ? YoloTensorType::FLOAT32 : YoloTensorType::FLOAT16;
}

}  // namespace

class YoloTensorRtBackend::Impl
{
public:
  explicit Impl(YoloTensorRtConfig config)
  : config_(std::move(config)), labels_(load_labels(config_.labels_path))
  {
    validate_config();
    session_ = std::make_unique<tensorrt::Session>(config_.engine_path);
    input_name_ = select_single_tensor(session_->input_names(), config_.input_tensor_name, "input");
    output_name_ =
      select_single_tensor(session_->output_names(), config_.output_tensor_name, "output");

    configure_input_shape();
    session_->prepare();
    input_buffer_ = session_->buffer(input_name_);
    output_shape_ = session_->tensor_shape(output_name_);
    preprocessor_ = create_yolo_preprocessor(config_.preprocessing_backend);
  }

  std::vector<Detection> detect(const ImageView & image)
  {
    std::scoped_lock lock(inference_mutex_);
    const YoloPreprocessDestination destination{
      input_buffer_.data, input_buffer_.size_bytes, yolo_tensor_type(input_buffer_.type),
      session_->stream()};
    const auto transform =
      preprocessor_->preprocess(image, input_width_, input_height_, destination);

    session_->enqueue();
    auto output = session_->download_output(output_name_);
    return postprocess_yolo(
      output, output_shape_, transform, image.width(), image.height(), labels_,
      config_.postprocess);
  }

private:
  void validate_config() const
  {
    if (
      config_.dynamic_input_width == 0U || config_.dynamic_input_height == 0U ||
      config_.dynamic_input_width >
        static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) ||
      config_.dynamic_input_height >
        static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
      throw std::invalid_argument("dynamic TensorRT input dimensions are invalid");
    }
  }

  void configure_input_shape()
  {
    auto input_shape = session_->tensor_shape(input_name_);
    if (input_shape.size() != 4U) {
      throw std::runtime_error(
        "YOLO TensorRT input must have NCHW shape, got " + shape_string(input_shape));
    }

    if (std::any_of(input_shape.begin(), input_shape.end(), [](auto value) { return value < 0; })) {
      input_shape[0] = input_shape[0] < 0 ? 1 : input_shape[0];
      input_shape[1] = input_shape[1] < 0 ? 3 : input_shape[1];
      input_shape[2] = input_shape[2] < 0 ? static_cast<std::int64_t>(config_.dynamic_input_height)
                                          : input_shape[2];
      input_shape[3] = input_shape[3] < 0 ? static_cast<std::int64_t>(config_.dynamic_input_width)
                                          : input_shape[3];
      session_->set_input_shape(input_name_, input_shape);
    }

    input_shape = session_->tensor_shape(input_name_);
    if (
      input_shape.size() != 4U || input_shape[0] != 1 || input_shape[1] != 3 ||
      input_shape[2] <= 0 || input_shape[3] <= 0) {
      throw std::runtime_error(
        "YOLO TensorRT input must resolve to [1,3,H,W], got " + shape_string(input_shape));
    }
    input_height_ = static_cast<std::size_t>(input_shape[2]);
    input_width_ = static_cast<std::size_t>(input_shape[3]);
  }

  YoloTensorRtConfig config_;
  std::vector<std::string> labels_;
  std::unique_ptr<tensorrt::Session> session_;
  std::unique_ptr<YoloPreprocessor> preprocessor_;
  tensorrt::TensorBufferView input_buffer_;
  std::mutex inference_mutex_;

  std::string input_name_;
  std::string output_name_;
  std::vector<std::int64_t> output_shape_;
  std::size_t input_width_{0U};
  std::size_t input_height_{0U};
};

#else

class YoloTensorRtBackend::Impl
{
public:
  explicit Impl(YoloTensorRtConfig)
  {
    throw std::runtime_error(
      "perception_detector was built without the TensorRT and CUDA development libraries");
  }

  std::vector<Detection> detect(const ImageView &)
  {
    return {};
  }
};

#endif

YoloTensorRtBackend::YoloTensorRtBackend(YoloTensorRtConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

YoloTensorRtBackend::~YoloTensorRtBackend() = default;
YoloTensorRtBackend::YoloTensorRtBackend(YoloTensorRtBackend &&) noexcept = default;
YoloTensorRtBackend & YoloTensorRtBackend::operator=(YoloTensorRtBackend &&) noexcept = default;

std::string YoloTensorRtBackend::name() const
{
  return "yolo_tensorrt";
}

std::vector<Detection> YoloTensorRtBackend::detect(const ImageView & image)
{
  return impl_->detect(image);
}

}  // namespace perception_detector
