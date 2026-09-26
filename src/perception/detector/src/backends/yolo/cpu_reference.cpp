#include "cpu_reference.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace perception_detector
{
namespace
{

constexpr float kLetterboxValue = 114.0F / 255.0F;

float read_rgb_channel(
  const ImageView & image, std::size_t row, std::size_t column, std::size_t channel)
{
  const auto * pixel =
    image.data() + row * image.row_step_bytes() + column * static_cast<std::size_t>(3U);
  const auto source_channel =
    image.encoding() == PixelEncoding::RGB8 ? channel : static_cast<std::size_t>(2U) - channel;
  return static_cast<float>(pixel[source_channel]) / 255.0F;
}

}  // namespace

CpuPreprocessedImage preprocess_yolo_cpu_reference(
  const ImageView & image, std::size_t input_width, std::size_t input_height)
{
  if (input_width == 0U || input_height == 0U) {
    throw std::invalid_argument("YOLO input dimensions must be positive");
  }
  if (
    input_width > std::numeric_limits<std::size_t>::max() / input_height ||
    input_width * input_height > std::numeric_limits<std::size_t>::max() / 3U) {
    throw std::overflow_error("YOLO input dimensions are too large");
  }

  const auto scale = std::min(
    static_cast<double>(input_width) / static_cast<double>(image.width()),
    static_cast<double>(input_height) / static_cast<double>(image.height()));
  const auto resized_width = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::llround(static_cast<double>(image.width()) * scale)));
  const auto resized_height = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::llround(static_cast<double>(image.height()) * scale)));
  const auto pad_left = (input_width - resized_width) / 2U;
  const auto pad_top = (input_height - resized_height) / 2U;

  CpuPreprocessedImage result;
  result.values.assign(3U * input_width * input_height, kLetterboxValue);
  result.transform = LetterboxTransform{
    input_width,
    input_height,
    static_cast<double>(resized_width) / static_cast<double>(image.width()),
    static_cast<double>(resized_height) / static_cast<double>(image.height()),
    static_cast<double>(pad_left),
    static_cast<double>(pad_top)};

  for (std::size_t destination_row = 0U; destination_row < resized_height; ++destination_row) {
    const auto source_y = std::clamp(
      (static_cast<double>(destination_row) + 0.5) / result.transform.scale_y - 0.5, 0.0,
      static_cast<double>(image.height() - 1U));
    const auto top = static_cast<std::size_t>(std::floor(source_y));
    const auto bottom = std::min(top + 1U, image.height() - 1U);
    const auto vertical_weight = source_y - static_cast<double>(top);

    for (std::size_t destination_column = 0U; destination_column < resized_width;
         ++destination_column) {
      const auto source_x = std::clamp(
        (static_cast<double>(destination_column) + 0.5) / result.transform.scale_x - 0.5, 0.0,
        static_cast<double>(image.width() - 1U));
      const auto left = static_cast<std::size_t>(std::floor(source_x));
      const auto right = std::min(left + 1U, image.width() - 1U);
      const auto horizontal_weight = source_x - static_cast<double>(left);
      const auto output_row = destination_row + pad_top;
      const auto output_column = destination_column + pad_left;

      for (std::size_t channel = 0U; channel < 3U; ++channel) {
        const auto top_left = read_rgb_channel(image, top, left, channel);
        const auto top_right = read_rgb_channel(image, top, right, channel);
        const auto bottom_left = read_rgb_channel(image, bottom, left, channel);
        const auto bottom_right = read_rgb_channel(image, bottom, right, channel);
        const auto top_value = top_left * (1.0 - horizontal_weight) + top_right * horizontal_weight;
        const auto bottom_value =
          bottom_left * (1.0 - horizontal_weight) + bottom_right * horizontal_weight;
        const auto value = top_value * (1.0 - vertical_weight) + bottom_value * vertical_weight;
        const auto output_index =
          channel * input_height * input_width + output_row * input_width + output_column;
        result.values[output_index] = static_cast<float>(value);
      }
    }
  }
  return result;
}

}  // namespace perception_detector
