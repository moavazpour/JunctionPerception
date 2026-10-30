#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

#include "jp/core/class_catalog.hpp"
#include "jp/detection/detector.hpp"
#include "jp/detection/letterbox.hpp"
#include "jp/detection/yolo_output.hpp"

namespace {

using jp::detection::Letterbox;

TEST(Letterbox, FitsWideImageWithVerticalPadding) {
  const auto lb = Letterbox::fit(cv::Size(1920, 1200), 1280);
  EXPECT_FLOAT_EQ(lb.scale, 1280.0F / 1920.0F);
  EXPECT_EQ(lb.resized_width, 1280);
  EXPECT_EQ(lb.resized_height, 800);
  EXPECT_EQ(lb.pad_left, 0);
  EXPECT_EQ(lb.pad_top, 240);
}

TEST(Letterbox, MapsBoxesBackToImagePixels) {
  const cv::Size image(1920, 1200);
  const auto lb = Letterbox::fit(image, 640);
  // An image box (300, 150)-(900, 600) maps into the input by scale and padding.
  const cv::Rect2f input_box(300 * lb.scale + static_cast<float>(lb.pad_left),
                             150 * lb.scale + static_cast<float>(lb.pad_top), 600 * lb.scale,
                             450 * lb.scale);
  const cv::Rect2f back = lb.to_image(input_box, image);
  EXPECT_NEAR(back.x, 300.0F, 1e-3F);
  EXPECT_NEAR(back.y, 150.0F, 1e-3F);
  EXPECT_NEAR(back.width, 600.0F, 1e-3F);
  EXPECT_NEAR(back.height, 450.0F, 1e-3F);
  // Boxes reaching into the padding are clipped to the image.
  const cv::Rect2f clipped = lb.to_image(cv::Rect2f(-10, 0, 100, 100), image);
  EXPECT_FLOAT_EQ(clipped.x, 0.0F);
  EXPECT_FLOAT_EQ(clipped.y, 0.0F);
}

TEST(Letterbox, ApplyProducesPaddedSquare) {
  const cv::Mat image(120, 200, CV_8UC3, cv::Scalar(10, 20, 30));
  const auto lb = Letterbox::fit(image.size(), 100);
  const cv::Mat input = lb.apply(image);
  ASSERT_EQ(input.size(), cv::Size(100, 100));
  EXPECT_EQ(input.at<cv::Vec3b>(0, 50), cv::Vec3b(114, 114, 114));
  EXPECT_EQ(input.at<cv::Vec3b>(50, 50), cv::Vec3b(10, 20, 30));
}

TEST(YoloOutput, ParsesRowsAboveThreshold) {
  const std::array<float, 18> output = {10, 20, 50, 80, 0.9F, 2,   // car
                                        5,  5,  15, 40, 0.3F, 0,   // person
                                        0,  0,  0,  0,  0.0F, 0};  // padding row
  const auto detections = jp::detection::parse_end2end_output(output, 0.25F);
  ASSERT_EQ(detections.size(), 2U);
  EXPECT_EQ(detections[0].class_id, 2);
  EXPECT_FLOAT_EQ(detections[0].box.width, 40.0F);
  EXPECT_FLOAT_EQ(detections[0].box.height, 60.0F);
  EXPECT_EQ(detections[1].class_id, 0);
  EXPECT_EQ(jp::detection::parse_end2end_output(output, 0.5F).size(), 1U);
  EXPECT_THROW(static_cast<void>(jp::detection::parse_end2end_output(
                   std::span<const float>(output.data(), 7), 0.1F)),
               std::invalid_argument);
}

TEST(YoloOutput, ParsesClassNamesMetadata) {
  const auto names = jp::detection::parse_class_names("{0: 'person', 1: 'bicycle', 2: 'car'}");
  EXPECT_EQ(names, (std::vector<std::string>{"person", "bicycle", "car"}));
  EXPECT_THROW(static_cast<void>(jp::detection::parse_class_names("{0: 'a', 2: 'c'}")),
               std::invalid_argument);
}

TEST(ClassMapping, KeepsMappedDetectionsOnly) {
  const auto catalog = jp::core::ClassCatalog::from_yaml(R"yaml(
classes:
  car: {length_m: 4.5, width_m: 1.8, height_m: 1.5, vru: false}
  pedestrian: {length_m: 0.5, width_m: 0.5, height_m: 1.5, vru: true}
sources:
  coco: {car: car, person: pedestrian}
)yaml");
  std::vector<jp::detection::Detection> detections(3);
  detections[0].source_label = "person";
  detections[1].source_label = "dog";
  detections[2].source_label = "car";
  const auto mapped = jp::detection::map_to_twin_classes(detections, catalog, "coco");
  ASSERT_EQ(mapped.size(), 2U);
  EXPECT_EQ(mapped[0].twin_class, "pedestrian");
  EXPECT_EQ(mapped[1].twin_class, "car");
}

}  // namespace
