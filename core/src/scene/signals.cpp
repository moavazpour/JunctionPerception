#include "jp/scene/signals.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace jp {

SignalState SignalReader::classify(const cv::Mat& crop) {
  SignalState s;
  if (crop.empty() || crop.cols < 3 || crop.rows < 3) return s;
  cv::Mat hsv;
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);
  double vmax = 0;
  cv::minMaxLoc(hsv.reshape(1, static_cast<int>(hsv.total())).col(2), nullptr, &vmax);
  const double vth = std::max(110.0, 0.6 * vmax);
  double r = 0, a = 0, g = 0, ry = 0, ay = 0, gy = 0;
  for (int y = 0; y < hsv.rows; ++y) {
    const auto* row = hsv.ptr<cv::Vec3b>(y);
    for (int x = 0; x < hsv.cols; ++x) {
      const int h = row[x][0], sat = row[x][1], val = row[x][2];
      if (val < vth || sat < 70) continue;
      const double w = val / 255.0, yn = (y + 0.5) / hsv.rows;
      if (h <= 10 || h >= 160) r += w, ry += w * yn;
      else if (h <= 35) a += w, ay += w * yn;
      else if (h >= 40 && h <= 100) g += w, gy += w * yn;
    }
  }
  const double total = r + a + g;
  const double area = static_cast<double>(crop.total());
  if (total < std::max(3.0, 0.002 * area)) {
    s.state = vmax < 90 ? "off" : "unknown";
    return s;
  }
  // Lamp order (top to bottom: red, amber, green) as a plausibility weight.
  auto place = [](double sum, double ysum, double lo, double hi) {
    if (sum <= 0) return 0.0;
    const double y = ysum / sum;
    return (y >= lo && y <= hi) ? 1.0 : 0.6;
  };
  const double R = r * place(r, ry, 0.0, 0.6), A = a * place(a, ay, 0.2, 0.8), G = g * place(g, gy, 0.4, 1.0);
  const double sum = R + A + G;
  if (R > 0.25 * sum && A > 0.25 * sum) {
    s.state = "red_amber";
  } else if (R >= A && R >= G) {
    s.state = "red";
  } else if (A >= G) {
    s.state = "amber";
  } else {
    s.state = "green";
  }
  s.confidence = std::max({R, A, G}) / sum * std::min(1.0, total / std::max(4.0, 0.01 * area));
  return s;
}

std::vector<SignalState> SignalReader::update(const cv::Mat& frame, const std::vector<SignalHead>& heads) {
  std::vector<SignalState> out;
  const cv::Rect bounds(0, 0, frame.cols, frame.rows);
  for (const auto& h : heads) {
    const cv::Rect r = cv::Rect(cv::Point(static_cast<int>(std::floor(std::min(h.box[0], h.box[2]))),
                                          static_cast<int>(std::floor(std::min(h.box[1], h.box[3])))),
                                cv::Point(static_cast<int>(std::ceil(std::max(h.box[0], h.box[2]))),
                                          static_cast<int>(std::ceil(std::max(h.box[1], h.box[3]))))) &
                       bounds;
    SignalState s = classify(r.area() > 0 ? frame(r) : cv::Mat());
    s.id = h.id;
    // Majority over the last 8 readings (about half a second).
    auto& hist = history_[h.id];
    hist.push_back(s.state);
    if (hist.size() > 8) hist.pop_front();
    std::map<std::string, int> votes;
    for (const auto& v : hist) votes[v]++;
    const auto best = std::max_element(votes.begin(), votes.end(), [](const auto& x, const auto& y) { return x.second < y.second; });
    const double share = static_cast<double>(best->second) / hist.size();
    if (best->first != s.state) s.state = best->first, s.confidence = 0;
    s.confidence = std::max(s.confidence, 0.0) * 0.5 + 0.5 * share;
    out.push_back(s);
  }
  return out;
}

}  // namespace jp
