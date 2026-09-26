#include "perception_detector/detector_backend.hpp"

#include <limits>

namespace perception_detector
{

std::optional<ImageView> ImageView::create(
  std::size_t width, std::size_t height, std::size_t row_step_bytes, PixelEncoding encoding,
  const std::uint8_t * data, std::size_t data_size_bytes)
{
  constexpr std::size_t channels = 3U;
  if (width == 0U || height == 0U || data == nullptr) {
    return std::nullopt;
  }
  if (width > std::numeric_limits<std::size_t>::max() / channels) {
    return std::nullopt;
  }

  const auto row_data_bytes = width * channels;
  if (row_step_bytes < row_data_bytes) {
    return std::nullopt;
  }
  if (height - 1U > std::numeric_limits<std::size_t>::max() / row_step_bytes) {
    return std::nullopt;
  }

  const auto last_row_offset = (height - 1U) * row_step_bytes;
  if (last_row_offset > data_size_bytes || row_data_bytes > data_size_bytes - last_row_offset) {
    return std::nullopt;
  }
  return ImageView{width, height, row_step_bytes, encoding, data};
}

ImageView::ImageView(
  std::size_t width, std::size_t height, std::size_t row_step_bytes, PixelEncoding encoding,
  const std::uint8_t * data)
: width_(width), height_(height), row_step_bytes_(row_step_bytes), encoding_(encoding), data_(data)
{
}

std::size_t ImageView::width() const
{
  return width_;
}

std::size_t ImageView::height() const
{
  return height_;
}

std::size_t ImageView::row_step_bytes() const
{
  return row_step_bytes_;
}

PixelEncoding ImageView::encoding() const
{
  return encoding_;
}

const std::uint8_t * ImageView::data() const
{
  return data_;
}

}  // namespace perception_detector
