#include "perception_detection_depth_projection/depth_projector.hpp"

#include <algorithm>
#include <cmath>

namespace perception_detection_depth_projection
{

DepthProjector::DepthProjector(DepthProjectorConfig config) : config_(config) {}

std::optional<Projection> DepthProjector::project(
  const BoundingBox2D & bbox, const DepthImage & depth, const CameraIntrinsics & intrinsics) const
{
  if (
    depth.width == 0U || depth.height == 0U ||
    depth.values_m.size() != depth.width * depth.height || intrinsics.fx <= 0.0 ||
    intrinsics.fy <= 0.0 || bbox.size_x <= 0.0 || bbox.size_y <= 0.0) {
    return std::nullopt;
  }

  const auto half_width = bbox.size_x * config_.center_roi_fraction / 2.0;
  const auto half_height = bbox.size_y * config_.center_roi_fraction / 2.0;
  const auto left = std::max(0, static_cast<int>(std::floor(bbox.center_x - half_width)));
  const auto right = std::min(
    static_cast<int>(depth.width) - 1, static_cast<int>(std::ceil(bbox.center_x + half_width)));
  const auto top = std::max(0, static_cast<int>(std::floor(bbox.center_y - half_height)));
  const auto bottom = std::min(
    static_cast<int>(depth.height) - 1, static_cast<int>(std::ceil(bbox.center_y + half_height)));
  if (left > right || top > bottom) {
    return std::nullopt;
  }

  std::vector<float> valid_depths;
  for (auto row = top; row <= bottom; ++row) {
    for (auto column = left; column <= right; ++column) {
      const auto value = depth.values_m[static_cast<std::size_t>(row) * depth.width + column];
      if (std::isfinite(value) && value >= config_.min_depth_m && value <= config_.max_depth_m) {
        valid_depths.push_back(value);
      }
    }
  }
  if (valid_depths.size() < config_.min_valid_samples) {
    return std::nullopt;
  }

  const auto median_index = valid_depths.size() / 2U;
  std::nth_element(valid_depths.begin(), valid_depths.begin() + median_index, valid_depths.end());
  const auto z_m = static_cast<double>(valid_depths[median_index]);

  return Projection{
    (bbox.center_x - intrinsics.cx) * z_m / intrinsics.fx,
    (bbox.center_y - intrinsics.cy) * z_m / intrinsics.fy,
    z_m,
    bbox.size_x * z_m / intrinsics.fx,
    bbox.size_y * z_m / intrinsics.fy,
  };
}

}  // namespace perception_detection_depth_projection
