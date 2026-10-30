#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <utility>
#include <vector>

namespace jp::io {

/// One decoded camera frame.
struct Frame {
  std::int64_t index = 0;         ///< 0-based position in the source.
  std::int64_t timestamp_ns = 0;  ///< Capture time; relative to the first frame for video files.
  cv::Mat image;                  ///< BGR, 8 bit.
};

/// Sequential source of camera frames (video file, image sequence; live streams later).
class FrameSource {
 public:
  FrameSource() = default;
  FrameSource(const FrameSource&) = delete;
  FrameSource& operator=(const FrameSource&) = delete;
  FrameSource(FrameSource&&) = delete;
  FrameSource& operator=(FrameSource&&) = delete;
  virtual ~FrameSource() = default;

  /// Next frame, or empty at the end of the source.
  [[nodiscard]] virtual std::optional<Frame> next() = 0;
  /// Average frame rate (frames per second).
  [[nodiscard]] virtual double fps() const = 0;
  /// Number of frames if known.
  [[nodiscard]] virtual std::optional<std::int64_t> frame_count() const = 0;
};

/// Video file decoded with OpenCV; timestamps are the presentation times of the frames, so
/// variable frame rate files keep their real timing.
class VideoFileSource final : public FrameSource {
 public:
  /// Throws std::runtime_error if the file cannot be opened.
  explicit VideoFileSource(const std::filesystem::path& path);
  ~VideoFileSource() override;
  VideoFileSource(const VideoFileSource&) = delete;
  VideoFileSource& operator=(const VideoFileSource&) = delete;
  VideoFileSource(VideoFileSource&&) = delete;
  VideoFileSource& operator=(VideoFileSource&&) = delete;

  [[nodiscard]] std::optional<Frame> next() override;
  [[nodiscard]] double fps() const override;
  [[nodiscard]] std::optional<std::int64_t> frame_count() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Ordered image files with given timestamps.
class ImageSequenceSource final : public FrameSource {
 public:
  /// `frames`: image paths with their timestamps (ns), in playback order.
  explicit ImageSequenceSource(std::vector<std::pair<std::filesystem::path, std::int64_t>> frames);

  /// All .jpg/.jpeg/.png files of a folder in name order, timestamps spaced at 1/fps.
  [[nodiscard]] static std::unique_ptr<ImageSequenceSource> from_directory(
      const std::filesystem::path& dir, double fps);

  [[nodiscard]] std::optional<Frame> next() override;
  [[nodiscard]] double fps() const override;
  [[nodiscard]] std::optional<std::int64_t> frame_count() const override;

 private:
  std::vector<std::pair<std::filesystem::path, std::int64_t>> frames_;
  std::size_t position_ = 0;
};

/// Opens a folder as image sequence (at `fps`) and any other path as video file.
[[nodiscard]] std::unique_ptr<FrameSource> open_frame_source(const std::filesystem::path& path,
                                                             double image_sequence_fps);

}  // namespace jp::io
