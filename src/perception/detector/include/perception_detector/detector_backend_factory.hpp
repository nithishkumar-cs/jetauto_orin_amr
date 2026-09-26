#ifndef PERCEPTION_DETECTOR__DETECTOR_BACKEND_FACTORY_HPP_
#define PERCEPTION_DETECTOR__DETECTOR_BACKEND_FACTORY_HPP_

#include <memory>

#include "perception_detector/detector_backend.hpp"

namespace rclcpp
{
class Node;
}

namespace perception_detector
{

// Selects and configures the production backend from node-scoped ROS parameters.
// Adding another production backend changes this factory, not DetectorNode.
std::unique_ptr<DetectorBackend> create_detector_backend(rclcpp::Node & node);

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__DETECTOR_BACKEND_FACTORY_HPP_
