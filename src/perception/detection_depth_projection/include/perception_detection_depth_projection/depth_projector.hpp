#ifndef PERCEPTION_DETECTION_DEPTH_PROJECTION__DEPTH_PROJECTOR_HPP_
#define PERCEPTION_DETECTION_DEPTH_PROJECTION__DEPTH_PROJECTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>

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

enum class DepthEncoding {
  FLOAT32_METERS,
  UINT16_MILLIMETERS,
};

// Non-owning view. The source buffer must remain alive while this view is used.
class DepthImageView
{
public:
  static std::optional<DepthImageView> create(
    std::size_t width, std::size_t height, std::size_t row_step_bytes, DepthEncoding encoding,
    bool is_big_endian, const std::uint8_t * data, std::size_t data_size_bytes);

  std::size_t width() const;
  std::size_t height() const;
  std::optional<float> value_m(std::size_t row, std::size_t column) const;

private:
  DepthImageView(
    std::size_t width, std::size_t height, std::size_t row_step_bytes, DepthEncoding encoding,
    bool is_big_endian, const std::uint8_t * data);

  std::size_t width_;
  std::size_t height_;
  std::size_t row_step_bytes_;
  DepthEncoding encoding_;
  bool is_big_endian_;
  const std::uint8_t * data_;
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
    const BoundingBox2D & bbox, const DepthImageView & depth,
    const CameraIntrinsics & intrinsics) const;

private:
  DepthProjectorConfig config_;
};

}  // namespace perception_detection_depth_projection

#endif  // PERCEPTION_DETECTION_DEPTH_PROJECTION__DEPTH_PROJECTOR_HPP_
