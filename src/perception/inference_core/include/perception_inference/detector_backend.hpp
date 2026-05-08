#pragma once

#include <amr_interfaces/msg/detection2_d.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <vector>

namespace perception_inference
{

struct DetectorConfig
{
  std::string backend{"debug"};
  std::string model_name{"debug_detector"};
  std::string engine_path;
  std::string labels_path;
  float confidence_threshold{0.25f};
  bool publish_debug_detections{true};
};

class DetectorBackend
{
public:
  virtual ~DetectorBackend() = default;

  virtual std::string id() const = 0;
  virtual bool ready() const = 0;
  virtual std::string status() const = 0;

  virtual std::vector<amr_interfaces::msg::Detection2D> detect(
    const sensor_msgs::msg::Image& image) = 0;
};

std::vector<std::string> load_labels(const std::string& path);
std::unique_ptr<DetectorBackend> make_detector_backend(const DetectorConfig& config);

}  // namespace perception_inference
