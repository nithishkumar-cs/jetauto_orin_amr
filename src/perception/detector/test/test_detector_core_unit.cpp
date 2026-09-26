#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "backends/yolo/cpu_reference.hpp"
#include "backends/yolo/postprocessing.hpp"
#include "perception_detector/detector_backend.hpp"

namespace perception_detector
{
namespace
{

ImageView make_image_view(
  const std::vector<std::uint8_t> & data, std::size_t width, std::size_t height,
  std::size_t row_step_bytes, PixelEncoding encoding = PixelEncoding::RGB8)
{
  return ImageView::create(width, height, row_step_bytes, encoding, data.data(), data.size())
    .value();
}

TEST(ImageViewUnit, AcceptsRowPaddingAndRejectsTruncatedLastRow)
{
  const std::vector<std::uint8_t> padded_data(16U, 0U);
  EXPECT_TRUE(
    ImageView::create(2U, 2U, 10U, PixelEncoding::RGB8, padded_data.data(), padded_data.size()));

  const std::vector<std::uint8_t> truncated_data(15U, 0U);
  EXPECT_FALSE(ImageView::create(
    2U, 2U, 10U, PixelEncoding::RGB8, truncated_data.data(), truncated_data.size()));
}

TEST(YoloPreprocessingUnit, LetterboxesBgrImageIntoRgbChwWithoutCopyingInput)
{
  // Two BGR pixels: red, then green.
  const std::vector<std::uint8_t> data{0U, 0U, 255U, 0U, 255U, 0U};
  const auto image = make_image_view(data, 2U, 1U, 6U, PixelEncoding::BGR8);

  const auto output = preprocess_yolo_cpu_reference(image, 4U, 4U);

  ASSERT_EQ(output.values.size(), 48U);
  EXPECT_DOUBLE_EQ(output.transform.scale_x, 2.0);
  EXPECT_DOUBLE_EQ(output.transform.scale_y, 2.0);
  EXPECT_DOUBLE_EQ(output.transform.pad_left, 0.0);
  EXPECT_DOUBLE_EQ(output.transform.pad_top, 1.0);
  EXPECT_FLOAT_EQ(output.values[0U], 114.0F / 255.0F);
  EXPECT_FLOAT_EQ(output.values[4U], 1.0F);   // R, first content row, first column.
  EXPECT_FLOAT_EQ(output.values[20U], 0.0F);  // G, same pixel.
  EXPECT_FLOAT_EQ(output.values[36U], 0.0F);  // B, same pixel.
}

TEST(YoloPostprocessingUnit, DecodesRawOutputAndAppliesClassAwareNms)
{
  // Shape [1, 6, 3]: xywh plus two class scores, stored attribute-major.
  const std::vector<float> output{
    50.0F, 52.0F, 50.0F,  // center x
    50.0F, 52.0F, 50.0F,  // center y
    20.0F, 20.0F, 20.0F,  // width
    20.0F, 20.0F, 20.0F,  // height
    0.90F, 0.80F, 0.10F,  // person
    0.10F, 0.20F, 0.95F,  // cart
  };
  const LetterboxTransform transform{100U, 100U, 1.0, 1.0, 0.0, 0.0};
  const YoloPostprocessConfig config{0.35, 0.45, 100U, YoloOutputFormat::RAW_XYWH};

  const auto detections =
    postprocess_yolo(output, {1, 6, 3}, transform, 100U, 100U, {"person", "cart"}, config);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_EQ(detections[0].class_id, "cart");
  EXPECT_NEAR(detections[0].score, 0.95, 1e-6);
  EXPECT_EQ(detections[1].class_id, "person");
  EXPECT_NEAR(detections[1].score, 0.90, 1e-6);
}

TEST(YoloPostprocessingUnit, MapsLetterboxedEndToEndBoxesBackToOriginalImage)
{
  const std::vector<float> output{10.0F, 30.0F, 60.0F, 55.0F, 0.8F, 0.0F};
  const LetterboxTransform transform{100U, 100U, 0.5, 0.5, 0.0, 25.0};
  const YoloPostprocessConfig config{0.35, 0.45, 100U, YoloOutputFormat::END_TO_END_XYXY};

  const auto detections =
    postprocess_yolo(output, {1, 1, 6}, transform, 200U, 100U, {"person"}, config);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_DOUBLE_EQ(detections[0].center_x, 70.0);
  EXPECT_DOUBLE_EQ(detections[0].center_y, 35.0);
  EXPECT_DOUBLE_EQ(detections[0].size_x, 100.0);
  EXPECT_DOUBLE_EQ(detections[0].size_y, 50.0);
}

TEST(YoloPostprocessingUnit, DoesNotApplyNmsTwiceToEndToEndOutput)
{
  const std::vector<float> output{
    10.0F, 10.0F, 60.0F, 60.0F, 0.9F, 0.0F, 12.0F, 12.0F, 58.0F, 58.0F, 0.8F, 0.0F,
  };
  const LetterboxTransform transform{100U, 100U, 1.0, 1.0, 0.0, 0.0};
  const YoloPostprocessConfig config{0.35, 0.1, 100U, YoloOutputFormat::END_TO_END_XYXY};

  const auto detections =
    postprocess_yolo(output, {1, 2, 6}, transform, 100U, 100U, {"person"}, config);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_NEAR(detections[0].score, 0.9, 1e-6);
  EXPECT_NEAR(detections[1].score, 0.8, 1e-6);
}

TEST(YoloPostprocessingUnit, DropsInvalidFinalBoxesBeforeCreatingDetections)
{
  const std::vector<float> output{
    10.0F,  10.0F, 20.0F,  20.0F, 0.9F, 0.0F,  // Valid.
    110.0F, 10.0F, 120.0F, 20.0F, 0.8F, 0.0F,  // Collapses at the right boundary.
    40.0F,  40.0F, 20.0F,  20.0F, 0.7F, 0.0F,  // Inverted corners.
  };
  const LetterboxTransform transform{100U, 100U, 1.0, 1.0, 0.0, 0.0};
  const YoloPostprocessConfig config{0.35, 0.45, 100U, YoloOutputFormat::END_TO_END_XYXY};

  const auto detections =
    postprocess_yolo(output, {1, 3, 6}, transform, 100U, 100U, {"person"}, config);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections.front().class_id, "person");
}

TEST(YoloPostprocessingUnit, RejectsOutputWhoseShapeDoesNotMatchItsBuffer)
{
  const LetterboxTransform transform{100U, 100U, 1.0, 1.0, 0.0, 0.0};
  EXPECT_THROW(
    postprocess_yolo({1.0F, 2.0F}, {1, 1, 6}, transform, 100U, 100U, {}, YoloPostprocessConfig{}),
    std::invalid_argument);
}

TEST(YoloPostprocessingUnit, ParsesOnlyDocumentedOutputFormats)
{
  EXPECT_EQ(yolo_output_format_from_string("raw_xywh"), YoloOutputFormat::RAW_XYWH);
  EXPECT_EQ(yolo_output_format_from_string("end_to_end_xyxy"), YoloOutputFormat::END_TO_END_XYXY);
  EXPECT_THROW(yolo_output_format_from_string("automatic"), std::invalid_argument);
}

}  // namespace
}  // namespace perception_detector
