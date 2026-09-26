#ifndef PERCEPTION_DETECTOR__DETECTOR_BACKEND_HPP_
#define PERCEPTION_DETECTOR__DETECTOR_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace perception_detector
{

enum class PixelEncoding {
  RGB8,
  BGR8,
};

// Non-owning view of one ROS image. The image message remains alive for the
// synchronous detector callback, so the backend can read without copying it.
class ImageView
{
public:
  static std::optional<ImageView> create(
    std::size_t width, std::size_t height, std::size_t row_step_bytes, PixelEncoding encoding,
    const std::uint8_t * data, std::size_t data_size_bytes);

  std::size_t width() const;
  std::size_t height() const;
  std::size_t row_step_bytes() const;
  PixelEncoding encoding() const;
  const std::uint8_t * data() const;

private:
  ImageView(
    std::size_t width, std::size_t height, std::size_t row_step_bytes, PixelEncoding encoding,
    const std::uint8_t * data);

  std::size_t width_;
  std::size_t height_;
  std::size_t row_step_bytes_;
  PixelEncoding encoding_;
  const std::uint8_t * data_;
};

// Backend-neutral result in the original input image's continuous pixel grid.
// A full-width box therefore spans x=[0, image_width], not [0, image_width-1].
struct Detection
{
  double center_x{0.0};
  double center_y{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double score{0.0};
  std::string class_id;
};

class DetectorBackend
{
public:
  virtual ~DetectorBackend() = default;

  virtual std::string name() const = 0;
  // Every returned detection must have finite coordinates, positive size, a
  // score in [0, 1], a non-empty class ID, and lie fully within image's grid.
  virtual std::vector<Detection> detect(const ImageView & image) = 0;
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__DETECTOR_BACKEND_HPP_
