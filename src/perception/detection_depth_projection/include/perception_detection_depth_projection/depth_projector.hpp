#ifndef PERCEPTION_GEOMETRY__DEPTH_PROJECTOR_HPP_
#define PERCEPTION_GEOMETRY__DEPTH_PROJECTOR_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace perception_detection_depth_projection
{

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct BoundingBox2D
{
  double center_x{0.0};
  double center_y{0.0};
  double size_x{0.0};
  double size_y{0.0};
};

struct DepthImage
{
  std::size_t width{0U};
  std::size_t height{0U};
  std::vector<float> values_m;
};

struct Projection
{
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
};

struct DepthProjectorConfig
{
  double center_roi_fraction{0.25};
  double min_depth_m{0.15};
  double max_depth_m{8.0};
  std::size_t min_valid_samples{5U};
};

class DepthProjector
{
public:
  explicit DepthProjector(DepthProjectorConfig config = {});

  std::optional<Projection> project(
    const BoundingBox2D & bbox, const DepthImage & depth,
    const CameraIntrinsics & intrinsics) const;

private:
  DepthProjectorConfig config_;
};

}  // namespace perception_detection_depth_projection

#endif  // PERCEPTION_GEOMETRY__DEPTH_PROJECTOR_HPP_
