#include "perception_detection_depth_projection/depth_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace perception_detection_depth_projection
{
namespace
{

std::size_t bytes_per_pixel(DepthEncoding encoding)
{
  switch (encoding) {
    case DepthEncoding::FLOAT32_METERS:
      return sizeof(float);
    case DepthEncoding::UINT16_MILLIMETERS:
      return sizeof(std::uint16_t);
  }
  return 0U;
}

std::uint16_t read_uint16(const std::uint8_t * data, bool is_big_endian)
{
  if (is_big_endian) {
    return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(data[0]) << 8U) | static_cast<std::uint16_t>(data[1]));
  }
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[0]) | (static_cast<std::uint16_t>(data[1]) << 8U));
}

std::uint32_t read_uint32(const std::uint8_t * data, bool is_big_endian)
{
  if (is_big_endian) {
    return (static_cast<std::uint32_t>(data[0]) << 24U) |
           (static_cast<std::uint32_t>(data[1]) << 16U) |
           (static_cast<std::uint32_t>(data[2]) << 8U) | static_cast<std::uint32_t>(data[3]);
  }
  return static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

float read_float32(const std::uint8_t * data, bool is_big_endian)
{
  const auto bits = read_uint32(data, is_big_endian);
  float value;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

std::optional<DepthImageView> DepthImageView::create(
  std::size_t width, std::size_t height, std::size_t row_step_bytes, DepthEncoding encoding,
  bool is_big_endian, const std::uint8_t * data, std::size_t data_size_bytes)
{
  const auto pixel_size = bytes_per_pixel(encoding);
  if (width == 0U || height == 0U || data == nullptr || pixel_size == 0U) {
    return std::nullopt;
  }
  if (width > std::numeric_limits<std::size_t>::max() / pixel_size) {
    return std::nullopt;
  }

  const auto row_data_size = width * pixel_size;
  if (row_step_bytes < row_data_size) {
    return std::nullopt;
  }
  if (height - 1U > std::numeric_limits<std::size_t>::max() / row_step_bytes) {
    return std::nullopt;
  }

  const auto final_row_offset = (height - 1U) * row_step_bytes;
  if (final_row_offset > data_size_bytes || row_data_size > data_size_bytes - final_row_offset) {
    return std::nullopt;
  }

  return DepthImageView{width, height, row_step_bytes, encoding, is_big_endian, data};
}

DepthImageView::DepthImageView(
  std::size_t width, std::size_t height, std::size_t row_step_bytes, DepthEncoding encoding,
  bool is_big_endian, const std::uint8_t * data)
: width_(width),
  height_(height),
  row_step_bytes_(row_step_bytes),
  encoding_(encoding),
  is_big_endian_(is_big_endian),
  data_(data)
{
}

std::size_t DepthImageView::width() const
{
  return width_;
}

std::size_t DepthImageView::height() const
{
  return height_;
}

std::optional<float> DepthImageView::value_m(std::size_t row, std::size_t column) const
{
  if (row >= height_ || column >= width_) {
    return std::nullopt;
  }

  const auto pixel_size = bytes_per_pixel(encoding_);
  const auto * pixel = data_ + row * row_step_bytes_ + column * pixel_size;
  if (encoding_ == DepthEncoding::UINT16_MILLIMETERS) {
    return static_cast<float>(read_uint16(pixel, is_big_endian_)) / 1000.0F;
  }

  return read_float32(pixel, is_big_endian_);
}

DepthProjector::DepthProjector(DepthProjectorConfig config) : config_(config) {}

std::optional<Projection> DepthProjector::project(
  const BoundingBox2D & bbox, const DepthImageView & depth,
  const CameraIntrinsics & intrinsics) const
{
  if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0 || bbox.size_x <= 0.0 || bbox.size_y <= 0.0) {
    return std::nullopt;
  }

  const auto half_width = bbox.size_x * config_.center_roi_fraction / 2.0;
  const auto half_height = bbox.size_y * config_.center_roi_fraction / 2.0;
  const auto left = std::max(0, static_cast<int>(std::floor(bbox.center_x - half_width)));
  const auto right = std::min(
    static_cast<int>(depth.width()) - 1, static_cast<int>(std::ceil(bbox.center_x + half_width)));
  const auto top = std::max(0, static_cast<int>(std::floor(bbox.center_y - half_height)));
  const auto bottom = std::min(
    static_cast<int>(depth.height()) - 1, static_cast<int>(std::ceil(bbox.center_y + half_height)));
  if (left > right || top > bottom) {
    return std::nullopt;
  }

  std::vector<float> valid_depths;
  for (auto row = top; row <= bottom; ++row) {
    for (auto column = left; column <= right; ++column) {
      const auto value = depth.value_m(row, column);
      if (
        value && std::isfinite(*value) && *value >= config_.min_depth_m &&
        *value <= config_.max_depth_m) {
        valid_depths.push_back(*value);
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
