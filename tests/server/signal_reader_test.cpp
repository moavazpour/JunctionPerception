#include "jp/sources/signal_reader.hpp"

#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

namespace {

using jp::sources::read_signal;

// A dark signal head (40 x 100 px) on a sky-grey background with three lamps; `lit` lamps
// (0: red, top; 1: yellow; 2: green, bottom) are drawn in their colour, the others dark.
cv::Mat signal_head(std::initializer_list<int> lit) {
  cv::Mat image(200, 200, CV_8UC3, cv::Scalar(170, 165, 160));
  cv::rectangle(image, cv::Rect(80, 50, 40, 100), cv::Scalar(30, 30, 30), cv::FILLED);
  const std::array<cv::Scalar, 3> colors = {cv::Scalar(40, 40, 250), cv::Scalar(20, 190, 255),
                                            cv::Scalar(120, 230, 40)};
  for (int lamp = 0; lamp < 3; ++lamp) {
    const bool on = std::find(lit.begin(), lit.end(), lamp) != lit.end();
    cv::circle(image, cv::Point(100, 66 + (lamp * 34)), 12,
               on ? colors.at(static_cast<std::size_t>(lamp)) : cv::Scalar(45, 45, 45), cv::FILLED);
  }
  return image;
}

const std::vector<cv::Point2f> kHead = {{78, 48}, {122, 48}, {122, 152}, {78, 152}};

TEST(SignalReader, ReadsTheLitLamp) {
  const jp::core::SignalSettings settings;
  EXPECT_EQ(read_signal(signal_head({0}), kHead, settings).state, "red");
  EXPECT_EQ(read_signal(signal_head({1}), kHead, settings).state, "yellow");
  EXPECT_EQ(read_signal(signal_head({2}), kHead, settings).state, "green");
  EXPECT_EQ(read_signal(signal_head({0, 1}), kHead, settings).state, "red_yellow");
  EXPECT_EQ(read_signal(signal_head({}), kHead, settings).state, "off");
  EXPECT_GT(read_signal(signal_head({2}), kHead, settings).confidence, 0.9);
}

TEST(SignalReader, ReadsAnOverexposedLamp) {
  // A red lamp too bright for the camera: a white core in a pale red halo.
  cv::Mat image = signal_head({});
  cv::circle(image, cv::Point(100, 66), 13, cv::Scalar(190, 190, 255), cv::FILLED);
  cv::circle(image, cv::Point(100, 66), 7, cv::Scalar(250, 250, 255), cv::FILLED);
  EXPECT_EQ(read_signal(image, kHead, {}).state, "red");
}

TEST(SignalReader, ReadsALampAgainstABrightSky) {
  // The area is mostly white sky and backplate around a small, washed-out green lamp.
  cv::Mat image(200, 200, CV_8UC3, cv::Scalar(240, 240, 238));
  cv::rectangle(image, cv::Rect(90, 40, 20, 50), cv::Scalar(40, 40, 40), cv::FILLED);
  cv::circle(image, cv::Point(100, 80), 9, cv::Scalar(200, 255, 190), cv::FILLED);
  cv::circle(image, cv::Point(100, 80), 4, cv::Scalar(250, 255, 250), cv::FILLED);
  const std::vector<cv::Point2f> area = {{70, 20}, {130, 20}, {130, 110}, {70, 110}};
  EXPECT_EQ(read_signal(image, area, {}).state, "green");
}

TEST(SignalReader, IgnoresTheBrightBackgroundOutsideTheArea) {
  // A red car outside the head does not count.
  cv::Mat image = signal_head({2});
  cv::rectangle(image, cv::Rect(130, 120, 60, 40), cv::Scalar(30, 30, 230), cv::FILLED);
  EXPECT_EQ(read_signal(image, kHead, {}).state, "green");
}

TEST(SignalFilter, ConfirmsChangesAndHoldsThroughFlicker) {
  jp::core::SignalSettings settings;
  settings.confirm_frames = 2;
  settings.hold_frames = 3;
  jp::sources::SignalFilter filter(settings);
  const jp::sources::SignalReading red{"red", 0.9};
  const jp::sources::SignalReading green{"green", 0.9};
  const jp::sources::SignalReading off{"off", 0.0};
  EXPECT_EQ(filter.update(red, 0).state, "unknown");  // one frame is not enough
  EXPECT_EQ(filter.update(red, 1).state, "red");
  EXPECT_EQ(filter.update(red, 1).since_ns, 1);
  // A single green frame (e.g. a reflection) does not switch.
  EXPECT_EQ(filter.update(green, 2).state, "red");
  EXPECT_EQ(filter.update(red, 3).state, "red");
  // LED flicker: a few unlit frames keep red.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(filter.update(off, 4 + i).state, "red");
  }
  EXPECT_EQ(filter.update(off, 7).state, "off");
  EXPECT_EQ(filter.update(green, 8).state, "off");
  const auto now = filter.update(green, 9);
  EXPECT_EQ(now.state, "green");
  EXPECT_EQ(now.since_ns, 9);
}

}  // namespace
