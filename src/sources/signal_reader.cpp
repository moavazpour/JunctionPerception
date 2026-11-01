#include "jp/sources/signal_reader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace jp::sources {

namespace {

// OpenCV hue runs from 0 to 180. Red wraps around 0; LED greens lean towards cyan.
constexpr int kRedLow = 10;
constexpr int kRedHigh = 165;
constexpr int kYellowLow = 11;
constexpr int kYellowHigh = 35;
constexpr int kGreenLow = 40;
constexpr int kGreenHigh = 100;
// Highest brightness threshold for lit pixels (HSV value).
constexpr int kMaxBrightThreshold = 235;

struct LitPixels {
  int area = 0;  ///< Pixels inside the polygon.
  int red = 0;
  int yellow = 0;
  int green = 0;
};

/// Median HSV value of the masked pixels: the brightness of the head without its lamps.
int median_value(const cv::Mat& hsv, const cv::Mat& mask, int pixels) {
  std::array<int, 256> histogram{};
  for (int y = 0; y < hsv.rows; ++y) {
    for (int x = 0; x < hsv.cols; ++x) {
      if (mask.at<unsigned char>(y, x) != 0) {
        ++histogram.at(hsv.at<cv::Vec3b>(y, x)[2]);
      }
    }
  }
  int seen = 0;
  for (std::size_t v = 0; v < histogram.size(); ++v) {
    seen += histogram.at(v);
    if (seen * 2 >= pixels) {
      return static_cast<int>(v);
    }
  }
  return 255;
}

/// Lit pixels per colour inside `polygon`.
LitPixels count_lit(const cv::Mat& bgr, const std::vector<cv::Point2f>& polygon,
                    const core::SignalSettings& settings) {
  std::vector<cv::Point> corners;
  corners.reserve(polygon.size());
  for (const auto& p : polygon) {
    corners.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  const cv::Rect area = cv::boundingRect(corners) & cv::Rect(0, 0, bgr.cols, bgr.rows);
  LitPixels lit;
  if (area.area() == 0) {
    return lit;
  }
  cv::Mat mask = cv::Mat::zeros(area.size(), CV_8U);
  for (auto& c : corners) {
    c -= area.tl();
  }
  cv::fillPoly(mask, std::vector{corners}, cv::Scalar(255));
  lit.area = cv::countNonZero(mask);
  if (lit.area == 0) {
    return lit;
  }
  cv::Mat hsv;
  cv::cvtColor(bgr(area), hsv, cv::COLOR_BGR2HSV);
  // Against bright sky or a white backplate the median is high already: the threshold stays
  // reachable, and saturation tells the lamp from the white surroundings.
  const int bright = std::min(
      std::max(settings.min_value, median_value(hsv, mask, lit.area) + settings.value_above_median),
      kMaxBrightThreshold);
  for (int y = 0; y < hsv.rows; ++y) {
    for (int x = 0; x < hsv.cols; ++x) {
      const auto& p = hsv.at<cv::Vec3b>(y, x);
      if (mask.at<unsigned char>(y, x) == 0 || p[2] < bright || p[1] < settings.min_saturation) {
        continue;
      }
      const int hue = p[0];
      if (hue <= kRedLow || hue >= kRedHigh) {
        ++lit.red;
      } else if (hue >= kYellowLow && hue <= kYellowHigh) {
        ++lit.yellow;
      } else if (hue >= kGreenLow && hue <= kGreenHigh) {
        ++lit.green;
      }
    }
  }
  return lit;
}

}  // namespace

SignalReading read_signal(const cv::Mat& bgr, const std::vector<cv::Point2f>& polygon,
                          const core::SignalSettings& settings) {
  SignalReading reading{"off"};
  if (bgr.empty() || polygon.size() < 3) {
    return reading;
  }
  LitPixels lit = count_lit(bgr, polygon, settings);
  if (lit.area == 0) {
    return reading;
  }
  const int needed = std::max(
      settings.min_lit_pixels, static_cast<int>(std::lround(settings.min_lit_fraction * lit.area)));
  if (std::max({lit.red, lit.yellow, lit.green}) < needed) {
    // An overexposed lamp has a white core and a pale halo: look again for less saturated,
    // less outstanding pixels.
    core::SignalSettings relaxed = settings;
    relaxed.min_saturation = settings.min_saturation / 2;
    relaxed.value_above_median = settings.value_above_median / 2;
    lit = count_lit(bgr, polygon, relaxed);
  }
  const double area = lit.area;
  reading.red = lit.red / area;
  reading.yellow = lit.yellow / area;
  reading.green = lit.green / area;
  const int best = std::max({lit.red, lit.yellow, lit.green});
  if (best < needed) {
    return reading;
  }
  const double total = lit.red + lit.yellow + lit.green;
  reading.confidence = best / total;
  if (best == lit.green) {
    reading.state = "green";
  } else if (lit.red >= needed && lit.yellow >= needed &&
             std::min(lit.red, lit.yellow) >= settings.pair_ratio * std::max(lit.red, lit.yellow)) {
    reading.state = "red_yellow";
    reading.confidence = (lit.red + lit.yellow) / total;
  } else {
    reading.state = best == lit.red ? "red" : "yellow";
  }
  return reading;
}

core::SignalState SignalFilter::update(const SignalReading& reading, std::int64_t timestamp_ns) {
  if (reading.state == "off") {
    // Unlit for a moment: keep the colour; for longer: the light is off or hidden.
    if (++unlit_frames_ > settings_.hold_frames && state_ != "off") {
      state_ = "off";
      confidence_ = 0.0;
      since_ns_ = timestamp_ns;
    }
    candidate_.clear();
    candidate_frames_ = 0;
  } else {
    unlit_frames_ = 0;
    if (reading.state == state_) {
      confidence_ = reading.confidence;
      candidate_.clear();
      candidate_frames_ = 0;
    } else {
      candidate_frames_ = reading.state == candidate_ ? candidate_frames_ + 1 : 1;
      candidate_ = reading.state;
      if (candidate_frames_ >= settings_.confirm_frames) {
        state_ = reading.state;
        confidence_ = reading.confidence;
        since_ns_ = timestamp_ns;
        candidate_.clear();
        candidate_frames_ = 0;
      }
    }
  }
  core::SignalState out;
  out.state = state_;
  out.confidence = confidence_;
  out.since_ns = since_ns_;
  return out;
}

}  // namespace jp::sources
