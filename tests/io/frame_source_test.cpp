#include "jp/io/frame_source.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <opencv2/imgcodecs.hpp>

namespace {

class ImageSequenceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() / "jp_frame_source_test";
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    for (int i = 0; i < 3; ++i) {
      const cv::Mat image(8, 12, CV_8UC3, cv::Scalar(i * 50, 0, 0));
      cv::imwrite((dir_ / ("frame_" + std::to_string(i) + ".png")).string(), image);
    }
    std::ofstream(dir_ / "notes.txt") << "not an image";
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::filesystem::path dir_;
};

TEST_F(ImageSequenceTest, ReadsImagesInOrderWithTimestamps) {
  const auto source = jp::io::open_frame_source(dir_, 10.0);
  ASSERT_EQ(source->frame_count(), 3);
  EXPECT_DOUBLE_EQ(source->fps(), 10.0);
  for (int i = 0; i < 3; ++i) {
    const auto frame = source->next();
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->index, i);
    EXPECT_EQ(frame->timestamp_ns, i * 100'000'000LL);
    EXPECT_EQ(frame->image.size(), cv::Size(12, 8));
    EXPECT_EQ(frame->image.at<cv::Vec3b>(0, 0)[0], i * 50);
  }
  EXPECT_FALSE(source->next().has_value());
}

TEST(VideoFileSource, MissingFileThrows) {
  EXPECT_THROW(jp::io::VideoFileSource("/nonexistent/video.mp4"), std::runtime_error);
}

}  // namespace
