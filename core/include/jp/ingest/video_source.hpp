#pragma once

#include <chrono>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>

namespace jp {

// Any camera behind one interface. URI forms:
//   file:///path/video.mp4   (or a bare path)  - optionally paced to real time and looped
//   rtsp://user:pass@host/stream               - IP cameras (ONVIF/RTSP), reconnects automatically
//   v4l2:///dev/video0 or usb:0                - USB / CSI via V4L2
//   gst:<pipeline ending in appsink>           - any GStreamer pipeline (GigE via aravis, Jetson nvarguscamerasrc,
//                                                NVDEC via nvv4l2decoder, ...)
struct VideoSourceConfig {
  std::string uri;
  bool realtime = true;   // pace file playback to its native frame rate
  bool loop = false;      // restart files at EOF
  double max_fps = 0;     // drop frames above this rate (0 = no limit)
  double reconnect_s = 2.0;
};

class VideoSource {
 public:
  explicit VideoSource(VideoSourceConfig cfg);
  // Blocks until the next frame. Returns false when the stream ended (non-looping files).
  bool read(cv::Mat& frame, double& stamp);
  double fps() const { return fps_; }
  cv::Size size() const { return size_; }
  bool isFile() const { return is_file_; }
  std::string describe() const;

 private:
  bool open();
  VideoSourceConfig cfg_;
  cv::VideoCapture cap_;
  bool is_file_ = false;
  double fps_ = 30;
  cv::Size size_;
  std::uint64_t frames_ = 0;
  double file_t0_ = 0;
  std::chrono::steady_clock::time_point wall0_;
  double last_emit_ = -1;
  bool paced_ = false;
};

}  // namespace jp
