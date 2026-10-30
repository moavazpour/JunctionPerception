#include "jp/io/frame_source.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>

namespace jp::io {

namespace {

constexpr double kNanosecondsPerMillisecond = 1e6;
constexpr double kNanosecondsPerSecond = 1e9;

bool is_image_file(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  std::ranges::transform(extension, extension.begin(),
                         [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  constexpr std::array kExtensions = {".jpg", ".jpeg", ".png"};
  return std::ranges::find(kExtensions, extension) != kExtensions.end();
}

}  // namespace

struct VideoFileSource::Impl {
  cv::VideoCapture capture;
  std::int64_t index = 0;
};

VideoFileSource::VideoFileSource(const std::filesystem::path& path)
    : impl_(std::make_unique<Impl>()) {
  if (!impl_->capture.open(path.string(), cv::CAP_FFMPEG)) {
    throw std::runtime_error("cannot open video " + path.string());
  }
}

VideoFileSource::~VideoFileSource() = default;

std::optional<Frame> VideoFileSource::next() {
  Frame frame;
  if (!impl_->capture.read(frame.image) || frame.image.empty()) {
    return std::nullopt;
  }
  frame.index = impl_->index++;
  frame.timestamp_ns = static_cast<std::int64_t>(
      std::llround(impl_->capture.get(cv::CAP_PROP_POS_MSEC) * kNanosecondsPerMillisecond));
  return frame;
}

double VideoFileSource::fps() const { return impl_->capture.get(cv::CAP_PROP_FPS); }

std::optional<std::int64_t> VideoFileSource::frame_count() const {
  const double count = impl_->capture.get(cv::CAP_PROP_FRAME_COUNT);
  if (count <= 0.0) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(count);
}

ImageSequenceSource::ImageSequenceSource(
    std::vector<std::pair<std::filesystem::path, std::int64_t>> frames)
    : frames_(std::move(frames)) {}

std::unique_ptr<ImageSequenceSource> ImageSequenceSource::from_directory(
    const std::filesystem::path& dir, double fps) {
  if (!std::filesystem::is_directory(dir)) {
    throw std::runtime_error("not a folder: " + dir.string());
  }
  if (fps <= 0.0) {
    throw std::invalid_argument("image sequence frame rate must be positive");
  }
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    if (entry.is_regular_file() && is_image_file(entry.path())) {
      files.push_back(entry.path());
    }
  }
  std::ranges::sort(files);
  std::vector<std::pair<std::filesystem::path, std::int64_t>> frames;
  frames.reserve(files.size());
  for (std::size_t i = 0; i < files.size(); ++i) {
    frames.emplace_back(files[i], static_cast<std::int64_t>(std::llround(
                                      static_cast<double>(i) * kNanosecondsPerSecond / fps)));
  }
  return std::make_unique<ImageSequenceSource>(std::move(frames));
}

std::optional<Frame> ImageSequenceSource::next() {
  if (position_ >= frames_.size()) {
    return std::nullopt;
  }
  const auto& [path, timestamp_ns] = frames_[position_];
  Frame frame;
  frame.index = static_cast<std::int64_t>(position_);
  frame.timestamp_ns = timestamp_ns;
  frame.image = cv::imread(path.string(), cv::IMREAD_COLOR);
  if (frame.image.empty()) {
    throw std::runtime_error("cannot read image " + path.string());
  }
  ++position_;
  return frame;
}

double ImageSequenceSource::fps() const {
  if (frames_.size() < 2) {
    return 0.0;
  }
  const double span_s =
      static_cast<double>(frames_.back().second - frames_.front().second) / kNanosecondsPerSecond;
  return span_s > 0.0 ? static_cast<double>(frames_.size() - 1) / span_s : 0.0;
}

std::optional<std::int64_t> ImageSequenceSource::frame_count() const {
  return static_cast<std::int64_t>(frames_.size());
}

std::unique_ptr<FrameSource> open_frame_source(const std::filesystem::path& path,
                                               double image_sequence_fps) {
  if (std::filesystem::is_directory(path)) {
    return ImageSequenceSource::from_directory(path, image_sequence_fps);
  }
  return std::make_unique<VideoFileSource>(path);
}

}  // namespace jp::io
