#include "jp/health/monitors.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sstream>

namespace jp {

std::string_view levelName(Level l) {
  switch (l) {
    case Level::Ok: return "ok";
    case Level::Degraded: return "degraded";
    default: return "fault";
  }
}

namespace {
constexpr double kRad2Deg = 180.0 / M_PI;
// Cityscapes train ids of static structures that can hide road users.
constexpr uint8_t Segmenter_Building = 2, Segmenter_Wall = 3, Segmenter_Fence = 4, Segmenter_Pole = 5,
                  Segmenter_TrafficLight = 6, Segmenter_TrafficSign = 7, Segmenter_Vegetation = 8, Segmenter_Sky = 10;

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  const size_t k = static_cast<size_t>(std::clamp(p, 0.0, 1.0) * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

std::string fmt(double v, int prec = 2) {
  std::ostringstream s;
  s.setf(std::ios::fixed);
  s.precision(prec);
  s << v;
  return s.str();
}
}  // namespace

// ================================================================== StabilityMonitor

void StabilityMonitor::setReference(const cv::Mat& gray, const std::vector<Eigen::Vector4d>& boxes, double scale) {
  cv::Mat mask(gray.size(), CV_8U, cv::Scalar(255));
  for (const auto& b : boxes) {
    const double w = (b[2] - b[0]) * 0.1, h = (b[3] - b[1]) * 0.1;
    cv::rectangle(mask, cv::Point2d((b[0] - w) * scale, (b[1] - h) * scale),
                  cv::Point2d((b[2] + w) * scale, (b[3] + h) * scale), cv::Scalar(0), cv::FILLED);
  }
  ref_ = gray.clone();
  ref_pts_.clear();
  cv::goodFeaturesToTrack(ref_, ref_pts_, cfg_.max_features, 0.01, gray.cols / 50.0, mask, 5);
}

void StabilityMonitor::rebase() {
  ref_.release();
  R_ref_cal_ = Eigen::Matrix3d::Identity();
  hist_.clear();
  drift_since_ = -1;
}

const StabilityMonitor::State& StabilityMonitor::update(double stamp, const cv::Mat& frame, const cv::Matx33d& K,
                                                        const std::vector<Eigen::Vector4d>& boxes) {
  scale_ = static_cast<double>(cfg_.width) / frame.cols;
  cv::Mat small, gray;
  cv::resize(frame, small, cv::Size(), scale_, scale_, cv::INTER_AREA);
  cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
  frames_++;
  if (ref_.empty() || ref_pts_.size() < 30) {
    setReference(gray, boxes, scale_);
    state_.status = frames_ < 3 ? "init" : state_.status;
    return state_;
  }
  std::vector<cv::Point2f> cur;
  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(ref_, gray, ref_pts_, cur, status, err, cv::Size(21, 21), 4);
  std::vector<cv::Point2f> a, b;
  for (size_t i = 0; i < cur.size(); ++i) {
    if (!status[i]) continue;
    bool on_object = false;
    for (const auto& bx : boxes) {
      const double x = cur[i].x / scale_, y = cur[i].y / scale_;
      if (x >= bx[0] && x <= bx[2] && y >= bx[1] && y <= bx[3]) {
        on_object = true;
        break;
      }
    }
    if (!on_object) {
      a.push_back(ref_pts_[i]);
      b.push_back(cur[i]);
    }
  }
  state_.track_ratio = static_cast<double>(a.size()) / std::max<size_t>(ref_pts_.size(), 1);
  if (a.size() < 12 || state_.track_ratio < cfg_.min_track_ratio) {
    state_.status = "lost";
    // Re-anchor after a while; the absolute pose is then unknown until recalibration.
    if (lost_since_ < 0) lost_since_ = stamp;
    if (stamp - lost_since_ > 5.0) setReference(gray, boxes, scale_);
    return state_;
  }
  lost_since_ = -1;
  std::vector<uchar> inl;
  cv::Mat H = cv::findHomography(a, b, cv::RANSAC, 1.5, inl);
  if (H.empty()) return state_;
  // Pure rotation: H = Ks R Ks^-1.
  Eigen::Matrix3d Ks, He;
  Ks << K(0, 0) * scale_, 0, K(0, 2) * scale_, 0, K(1, 1) * scale_, K(1, 2) * scale_, 0, 0, 1;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) He(r, c) = H.at<double>(r, c);
  Eigen::Matrix3d R = Ks.inverse() * He * Ks;
  R /= std::cbrt(R.determinant());
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0) R = -R;
  const Eigen::Matrix3d R_cur_cal = R * R_ref_cal_;
  state_.R_cur_ref = R_cur_cal;
  const Eigen::AngleAxisd aa(R_cur_cal);
  const Eigen::Vector3d rv = aa.axis() * aa.angle() * kRad2Deg;
  hist_.emplace_back(stamp, rv);
  while (!hist_.empty() && stamp - hist_.front().first > 3.0) hist_.pop_front();

  // Shake = RMS deviation from a 1 s moving mean; drift = magnitude of the 1 s mean.
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  int n = 0;
  for (const auto& [t, v] : hist_)
    if (stamp - t <= 1.0) {
      mean += v;
      ++n;
    }
  mean /= std::max(n, 1);
  double ss = 0;
  for (const auto& [t, v] : hist_) ss += (v - mean).squaredNorm();
  const double jitter_deg = std::sqrt(ss / std::max<size_t>(hist_.size(), 1));
  state_.jitter_px = jitter_deg / kRad2Deg * K(0, 0);
  state_.drift_deg = mean.norm();
  const double shake_full = cfg_.shake_px / scale_;
  state_.compensating = state_.drift_deg > 0.05 || state_.jitter_px > 0.5;
  if (state_.drift_deg > cfg_.drift_deg) {
    if (drift_since_ < 0) drift_since_ = stamp;
  } else {
    drift_since_ = -1;
  }
  if (state_.drift_deg > cfg_.max_compensate_deg) {
    state_.status = "moved";
  } else if (state_.jitter_px > shake_full) {
    state_.status = "shaking";
  } else if (drift_since_ >= 0 && stamp - drift_since_ > cfg_.drift_hold_s) {
    state_.status = "drifted";  // compensated, but the calibration should be re-anchored
  } else {
    state_.status = "stable";
  }
  // Refresh the reference when features fade (lighting changes) while the camera is calm.
  if (state_.track_ratio < 0.5 && state_.status == "stable") {
    R_ref_cal_ = R_cur_cal;
    setReference(gray, boxes, scale_);
  }
  return state_;
}

// ================================================================== VisibilityMonitor

const VisibilityMonitor::State& VisibilityMonitor::update(double stamp, const cv::Mat& frame,
                                                          const std::vector<Track>& tracks) {
  if (t0_ < 0) t0_ = stamp;
  const bool learning = stamp - t0_ < cfg_.baseline_s;
  cv::Mat small, gray;
  const double s = 320.0 / frame.cols;
  cv::resize(frame, small, cv::Size(), s, s, cv::INTER_AREA);
  cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);

  cv::Scalar m, sd;
  cv::meanStdDev(gray, m, sd);
  st_.brightness = m[0];
  st_.contrast = sd[0] / 128.0;
  st_.saturated = cv::countNonZero(gray >= 250) / static_cast<double>(gray.total());
  st_.dark = cv::countNonZero(gray <= 20) / static_cast<double>(gray.total());

  cv::Mat lap;
  cv::Laplacian(gray, lap, CV_32F);
  cv::meanStdDev(lap, m, sd);
  const double sharp = sd[0] * sd[0];
  // Dark channel prior: haze lifts the minimum intensity of local patches.
  std::vector<cv::Mat> ch;
  cv::split(small, ch);
  cv::Mat dc = cv::min(cv::min(ch[0], ch[1]), ch[2]);
  cv::erode(dc, dc, cv::getStructuringElement(cv::MORPH_RECT, {7, 7}));
  st_.haze = cv::mean(dc)[0] / 255.0;
  // Contrast far (upper third) vs. near (lower third): fog attenuates distant contrast first.
  cv::Scalar mf, sf, mn, sn;
  cv::meanStdDev(gray.rowRange(gray.rows / 6, gray.rows / 2), mf, sf);
  cv::meanStdDev(gray.rowRange(2 * gray.rows / 3, gray.rows), mn, sn);
  const double far_contrast = sf[0] / std::max(sn[0], 1.0);

  // Local sharpness grid for lens obstruction (droplets, dirt, condensation).
  cv::Mat cells(cfg_.grid_rows, cfg_.grid_cols, CV_64F);
  const int cw = gray.cols / cfg_.grid_cols, chh = gray.rows / cfg_.grid_rows;
  for (int r = 0; r < cfg_.grid_rows; ++r)
    for (int c = 0; c < cfg_.grid_cols; ++c) {
      cv::Scalar cm, cs;
      cv::meanStdDev(lap(cv::Rect(c * cw, r * chh, cw, chh)), cm, cs);
      cells.at<double>(r, c) = cs[0] * cs[0];
    }
  if (base_cells_.empty()) {
    base_cells_ = cells.clone();
    bad_since_ = cv::Mat(cells.size(), CV_64F, cv::Scalar(-1));
  }
  if (learning) {
    const double a = 1.0 / (++base_n_);
    base_sharp_ = (1 - a) * base_sharp_ + a * sharp;
    base_far_contrast_ = (1 - a) * base_far_contrast_ + a * far_contrast;
    base_cells_ = (1 - a) * base_cells_ + a * cells;
  }
  st_.sharpness_ratio = base_sharp_ > 0 ? sharp / base_sharp_ : 1.0;
  st_.far_contrast_ratio = base_far_contrast_ > 0 ? far_contrast / base_far_contrast_ : 1.0;
  int obstructed = 0;
  for (int r = 0; r < cells.rows; ++r)
    for (int c = 0; c < cells.cols; ++c) {
      const double ratio = cells.at<double>(r, c) / std::max(base_cells_.at<double>(r, c), 1e-6);
      double& since = bad_since_.at<double>(r, c);
      if (!learning && ratio < 0.25) {
        if (since < 0) since = stamp;
      } else {
        since = -1;
        if (!learning && ratio > 0.6) base_cells_.at<double>(r, c) = 0.999 * base_cells_.at<double>(r, c) + 0.001 * cells.at<double>(r, c);
      }
      if (since >= 0 && stamp - since > cfg_.obstruction_hold_s) ++obstructed;
    }
  st_.obstructed_fraction = static_cast<double>(obstructed) / cells.total();

  // Frozen stream: the picture does not change at all.
  if (!prev_small_.empty()) {
    cv::Mat d;
    cv::absdiff(gray, prev_small_, d);
    if (cv::mean(d)[0] < 0.2) {
      if (still_since_ < 0) still_since_ = stamp;
    } else {
      still_since_ = -1;
    }
  }
  prev_small_ = gray.clone();
  st_.frozen = still_since_ >= 0 && stamp - still_since_ > cfg_.frozen_s;

  // Detection horizon: range at which tracks are first confirmed; fragmentation of short tracks.
  std::set<std::uint32_t> alive;
  for (const auto& t : tracks) {
    if (t.state == TrackState::Tentative) continue;
    alive.insert(t.id);
    if (!first_seen_.count(t.id) && isVehicle(t.cls)) {
      first_seen_[t.id] = {stamp, t.last_range};
      horizon_.emplace_back(stamp, t.last_range);
    }
    last_seen_[t.id] = stamp;
  }
  for (auto it = first_seen_.begin(); it != first_seen_.end();) {
    if (!alive.count(it->first)) {
      lifetimes_.emplace_back(stamp, last_seen_[it->first] - it->second.first);
      last_seen_.erase(it->first);
      it = first_seen_.erase(it);
    } else {
      ++it;
    }
  }
  while (!horizon_.empty() && stamp - horizon_.front().first > 600) horizon_.pop_front();
  while (!lifetimes_.empty() && stamp - lifetimes_.front().first > 300) lifetimes_.pop_front();
  std::vector<double> rs;
  for (const auto& [t, r] : horizon_) rs.push_back(r);
  st_.effective_range = rs.size() >= 5 ? percentile(rs, 0.9) : 0;
  if (learning || st_.effective_range > st_.baseline_range) st_.baseline_range = std::max(st_.baseline_range, st_.effective_range);
  int short_tracks = 0;
  for (const auto& [t, life] : lifetimes_) short_tracks += life < 1.0;
  st_.fragmentation = lifetimes_.empty() ? 0 : static_cast<double>(short_tracks) / lifetimes_.size();

  st_.conditions.clear();
  double score = 1.0;
  if (st_.brightness < 55) {
    st_.conditions.push_back("night");
    score *= 0.85;
  }
  if (st_.saturated > 0.04) {
    st_.conditions.push_back("glare");
    score *= 0.8;
  }
  if (!learning && st_.far_contrast_ratio < 0.55 && st_.haze > 0.35) {
    st_.conditions.push_back("fog");
    score *= 0.6;
  }
  if (st_.contrast < 0.10) {
    st_.conditions.push_back("low_contrast");
    score *= 0.8;
  }
  if (!learning && st_.sharpness_ratio < 0.4) {
    st_.conditions.push_back("blur");
    score *= 0.6;
  }
  if (st_.obstructed_fraction > 0.08) {
    st_.conditions.push_back("lens_obstruction");
    score *= std::max(0.2, 1.0 - 2.0 * st_.obstructed_fraction);
  }
  if (st_.frozen) {
    st_.conditions.push_back("frozen");
    score = 0;
  }
  if (st_.baseline_range > 0 && st_.effective_range > 0)
    score *= std::clamp(st_.effective_range / (0.7 * st_.baseline_range), 0.3, 1.0);
  if (st_.fragmentation > 0.35) score *= 0.8;
  st_.score = score;
  return st_;
}

// ================================================================== CalibrationMonitor

const CalibrationMonitor::State& CalibrationMonitor::update(double stamp, const CameraModel& cam,
                                                            const std::vector<Detection3D>& dets,
                                                            const std::vector<RoadUser>& users, const SiteMap& map) {
  for (const auto& d : dets) {
    const auto& b = d.img.box2d;
    const double h2d = b[3] - b[1];
    if (d.img.score < 0.5 || h2d < 25 || d.range > 120 || !isVehicle(d.img.cls)) continue;
    if (b[0] < 5 || b[1] < 5 || b[2] > cam.width() - 5 || b[3] > cam.height() - 5) continue;
    auto px = cam.projectBox(d.bottom, d.yaw, d.dims);
    if (!px) continue;
    double y0 = 1e9, y1 = -1e9;
    for (const auto& p : *px) {
      y0 = std::min(y0, p.y());
      y1 = std::max(y1, p.y());
    }
    samples_.push_back({stamp, d.range, (y1 - y0) / h2d});
  }
  for (const auto& u : users) {
    if (u.lane.empty() || u.speed < 4.0 || !isVehicle(u.cls)) continue;
    const Lane* l = map.lane(u.lane);
    if (!l) continue;
    const auto p = l->centerline.project(u.pos);
    heading_err_.emplace_back(stamp, std::abs(wrapAngle(u.yaw - p.heading)) * kRad2Deg);
  }
  for (const auto& u : users)
    if (isVehicle(u.cls) && u.state == TrackState::Confirmed && u.age > 2.0) speeds_.emplace_back(stamp, u.speed);
  while (!samples_.empty() && stamp - samples_.front().t > 300) samples_.pop_front();
  while (!heading_err_.empty() && stamp - heading_err_.front().first > 300) heading_err_.pop_front();
  while (!speeds_.empty() && stamp - speeds_.front().first > 300) speeds_.pop_front();

  st_.samples = static_cast<int>(samples_.size());
  st_.reasons.clear();
  if (samples_.size() < 40) {
    st_.status = "unknown";
    st_.score = 0.8;
    return st_;
  }
  std::vector<double> ratios;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (const auto& s : samples_) {
    ratios.push_back(s.ratio);
    const double x = s.range / 100.0;
    sx += x;
    sy += s.ratio;
    sxx += x * x;
    sxy += x * s.ratio;
  }
  const double n = samples_.size();
  st_.size_ratio = percentile(ratios, 0.5);
  const double den = n * sxx - sx * sx;
  st_.size_slope = std::abs(den) > 1e-9 ? (n * sxy - sx * sy) / den : 0.0;
  std::vector<double> he;
  for (const auto& [t, e] : heading_err_) he.push_back(e);
  st_.heading_error_deg = he.size() > 20 ? percentile(he, 0.5) : 0.0;
  std::vector<double> sp;
  for (const auto& [t, v] : speeds_) sp.push_back(v);
  st_.speed_p95 = percentile(sp, 0.95);

  const double size_err = std::abs(st_.size_ratio - 1.0);
  bool invalid = size_err > 0.3 || st_.heading_error_deg > 15;
  bool suspect = size_err > 0.15 || std::abs(st_.size_slope) > 0.25 || st_.heading_error_deg > 8 || st_.speed_p95 > 30;
  if (size_err > 0.15) st_.reasons.push_back("object size mismatch " + fmt(st_.size_ratio));
  if (std::abs(st_.size_slope) > 0.25) st_.reasons.push_back("range-dependent size bias " + fmt(st_.size_slope));
  if (st_.heading_error_deg > 8) st_.reasons.push_back("track/lane heading error " + fmt(st_.heading_error_deg, 1) + " deg");
  if (st_.speed_p95 > 30) st_.reasons.push_back("implausible speeds p95 " + fmt(st_.speed_p95, 1) + " m/s");
  st_.status = invalid ? "invalid" : suspect ? "suspect" : "ok";
  st_.score = std::clamp(1.0 - 1.5 * size_err - 0.5 * std::abs(st_.size_slope) - st_.heading_error_deg / 40.0, 0.0, 1.0);
  return st_;
}

// ================================================================== OcclusionMap

void OcclusionMap::configure(const CameraModel& cam, const SiteMap& map, double default_range) {
  map_ = &map;
  roi_ = map.roi;
  if (roi_.empty()) {
    // Fallback region: a wedge in front of the camera.
    const double b = cam.bearingDeg() * M_PI / 180.0;
    const Eigen::Vector2d f(std::sin(b), std::cos(b)), l(-f.y(), f.x());
    roi_ = {f * 3 + l * 25, f * default_range + l * default_range * 0.4, f * default_range - l * default_range * 0.4,
            f * 3 - l * 25};
    for (const auto& lane : map.lanes)
      for (const auto& p : lane.centerline.pts) roi_.push_back(p);
    // Use the bounding polygon of everything (convex hull).
    std::vector<cv::Point2f> pts, hull;
    for (const auto& p : roi_) pts.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
    cv::convexHull(pts, hull);
    roi_.clear();
    for (const auto& p : hull) roi_.emplace_back(p.x, p.y);
  }
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const auto& p : roi_) {
    x0 = std::min(x0, p.x());
    y0 = std::min(y0, p.y());
    x1 = std::max(x1, p.x());
    y1 = std::max(y1, p.y());
  }
  origin_ = {x0, y0};
  cols_ = std::min(600, static_cast<int>(std::ceil((x1 - x0) / cfg_.cell)));
  rows_ = std::min(600, static_cast<int>(std::ceil((y1 - y0) / cfg_.cell)));
  base_.assign(static_cast<size_t>(cols_) * rows_, OutOfView);
  static_prob_.assign(base_.size(), 0.f);
  occluder_.assign(base_.size(), 0);
  for (int r = 0; r < rows_; ++r)
    for (int c = 0; c < cols_; ++c) {
      const Eigen::Vector2d p = origin_ + Eigen::Vector2d((c + 0.5) * cfg_.cell, (r + 0.5) * cfg_.cell);
      if (!pointInPolygon(roi_, p)) continue;
      auto g = cam.groundToPixel({p.x(), p.y(), 0.0});
      auto t = cam.groundToPixel({p.x(), p.y(), 1.7});
      const auto inside = [&](const std::optional<Eigen::Vector2d>& q) {
        return q && q->x() >= 0 && q->y() >= 0 && q->x() < cam.width() && q->y() < cam.height();
      };
      uint8_t s = OutOfView;
      if (inside(g) || inside(t)) s = (g && t && std::abs(g->y() - t->y()) < cfg_.min_ped_px) ? BeyondResolution : Visible;
      base_[static_cast<size_t>(r) * cols_ + c] = s;
    }
  state_ = base_;
}

void OcclusionMap::updateStatic(const cv::Mat& labels, const CameraModel& cam) {
  if (base_.empty() || labels.empty()) return;
  const double sx = static_cast<double>(labels.cols) / cam.width(), sy = static_cast<double>(labels.rows) / cam.height();
  auto isStatic = [](uint8_t l) {
    return l == Segmenter_Building || l == Segmenter_Wall || l == Segmenter_Fence || l == Segmenter_Pole ||
           l == Segmenter_TrafficLight || l == Segmenter_TrafficSign || l == Segmenter_Vegetation || l == Segmenter_Sky;
  };
  auto labelAt = [&](const std::optional<Eigen::Vector2d>& q) -> int {
    if (!q) return -1;
    const int x = static_cast<int>(q->x() * sx), y = static_cast<int>(q->y() * sy);
    if (x < 0 || y < 0 || x >= labels.cols || y >= labels.rows) return -1;
    return labels.at<uint8_t>(y, x);
  };
  for (int r = 0; r < rows_; ++r)
    for (int c = 0; c < cols_; ++c) {
      const size_t i = static_cast<size_t>(r) * cols_ + c;
      if (base_[i] != Visible) continue;
      const Eigen::Vector2d p = origin_ + Eigen::Vector2d((c + 0.5) * cfg_.cell, (r + 0.5) * cfg_.cell);
      const int lg = labelAt(cam.groundToPixel({p.x(), p.y(), 0.0}));
      const int lp = labelAt(cam.groundToPixel({p.x(), p.y(), cfg_.probe_height}));
      const bool hidden = (lg < 0 || isStatic(static_cast<uint8_t>(lg))) && (lp < 0 || isStatic(static_cast<uint8_t>(lp)));
      static_prob_[i] = 0.95f * static_prob_[i] + 0.05f * (hidden ? 1.f : 0.f);
    }
}

void OcclusionMap::updateDynamic(const CameraModel& cam, const std::vector<Track>& tracks) {
  if (base_.empty()) return;
  struct Occ {
    std::vector<cv::Point2f> hull;
    cv::Rect2f rect;
    double range;
    std::uint32_t id;
  };
  std::vector<Occ> occs;
  for (const auto& t : tracks) {
    if (t.state == TrackState::Tentative || t.dims[2] < 1.0) continue;
    auto px = cam.projectBox({t.position().x(), t.position().y(), 0.0}, t.yaw(), t.dims);
    if (!px) continue;
    std::vector<cv::Point2f> pts;
    for (const auto& p : *px) pts.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
    Occ o;
    cv::convexHull(pts, o.hull);
    o.rect = cv::boundingRect(o.hull);
    o.range = t.position().norm() - t.dims[0] / 2;
    o.id = t.id;
    occs.push_back(std::move(o));
  }
  for (int r = 0; r < rows_; ++r)
    for (int c = 0; c < cols_; ++c) {
      const size_t i = static_cast<size_t>(r) * cols_ + c;
      uint8_t s = base_[i];
      occluder_[i] = 0;
      if (s == Visible && static_prob_[i] > 0.6f) s = StaticOccluded;
      if (s == Visible && !occs.empty()) {
        const Eigen::Vector2d p = origin_ + Eigen::Vector2d((c + 0.5) * cfg_.cell, (r + 0.5) * cfg_.cell);
        const double range = p.norm();
        auto q = cam.groundToPixel({p.x(), p.y(), cfg_.probe_height});
        if (q) {
          const cv::Point2f qp(static_cast<float>(q->x()), static_cast<float>(q->y()));
          for (const auto& o : occs) {
            if (range <= o.range + 1.0 || !o.rect.contains(qp)) continue;
            if (cv::pointPolygonTest(o.hull, qp, false) > 0) {
              s = DynamicOccluded;
              occluder_[i] = o.id;
              break;
            }
          }
        }
      }
      state_[i] = s;
    }
  if (map_) summarize(*map_);
}

bool OcclusionMap::occluded(const Eigen::Vector2d& p) const {
  if (state_.empty()) return false;
  const int c = static_cast<int>((p.x() - origin_.x()) / cfg_.cell), r = static_cast<int>((p.y() - origin_.y()) / cfg_.cell);
  if (c < 0 || r < 0 || c >= cols_ || r >= rows_) return false;
  const uint8_t s = state_[static_cast<size_t>(r) * cols_ + c];
  return s == StaticOccluded || s == DynamicOccluded;
}

void OcclusionMap::summarize(const SiteMap& map) {
  coverage_.clear();
  segments_.clear();
  auto cellAt = [&](const Eigen::Vector2d& p, std::uint32_t& occ) -> int {
    const int c = static_cast<int>((p.x() - origin_.x()) / cfg_.cell), r = static_cast<int>((p.y() - origin_.y()) / cfg_.cell);
    if (c < 0 || r < 0 || c >= cols_ || r >= rows_) return OutOfView;
    const size_t i = static_cast<size_t>(r) * cols_ + c;
    occ = occluder_[i];
    return state_[i];
  };
  auto causeOf = [](int s, std::uint32_t occ) -> std::string {
    switch (s) {
      case StaticOccluded: return "static";
      case DynamicOccluded: return "vehicle:" + std::to_string(occ);
      case BeyondResolution: return "resolution";
      default: return "out_of_view";
    }
  };
  for (const auto& l : map.lanes) {
    ZoneCoverage zc;
    zc.zone = l.id;
    const double L = l.centerline.length();
    int n = 0, vis = 0, st = 0, dy = 0, un = 0;
    std::string cur;
    double seg0 = 0;
    for (double s = 0; s <= L; s += 1.0) {
      std::uint32_t occ = 0;
      const int c = cellAt(l.centerline.at(s), occ);
      ++n;
      vis += c == Visible;
      st += c == StaticOccluded;
      dy += c == DynamicOccluded;
      un += c == OutOfView || c == BeyondResolution;
      const std::string cause = c == Visible ? "" : causeOf(c, occ);
      if (cause != cur) {
        if (!cur.empty() && s - seg0 >= 2.0) segments_.push_back({l.id, seg0, s, cur});
        cur = cause;
        seg0 = s;
      }
    }
    if (!cur.empty() && L - seg0 >= 2.0) segments_.push_back({l.id, seg0, L, cur});
    if (n) {
      zc.visible = static_cast<double>(vis) / n;
      zc.static_occluded = static_cast<double>(st) / n;
      zc.dynamic_occluded = static_cast<double>(dy) / n;
      zc.unobservable = static_cast<double>(un) / n;
    }
    coverage_.push_back(zc);
  }
  int k = 0;
  for (const auto& cw : map.crosswalks) {
    ZoneCoverage zc;
    zc.zone = "crosswalk" + std::to_string(++k);
    int n = 0, vis = 0, st = 0, dy = 0, un = 0;
    for (int r = 0; r < rows_; ++r)
      for (int c = 0; c < cols_; ++c) {
        const Eigen::Vector2d p = origin_ + Eigen::Vector2d((c + 0.5) * cfg_.cell, (r + 0.5) * cfg_.cell);
        if (!pointInPolygon(cw, p)) continue;
        const uint8_t s = state_[static_cast<size_t>(r) * cols_ + c];
        ++n;
        vis += s == Visible;
        st += s == StaticOccluded;
        dy += s == DynamicOccluded;
        un += s == OutOfView || s == BeyondResolution;
      }
    if (n) {
      zc.visible = static_cast<double>(vis) / n;
      zc.static_occluded = static_cast<double>(st) / n;
      zc.dynamic_occluded = static_cast<double>(dy) / n;
      zc.unobservable = static_cast<double>(un) / n;
    }
    coverage_.push_back(zc);
  }
}

// ================================================================== integrity

HealthReport assessIntegrity(const StabilityMonitor::State& stab, const VisibilityMonitor::State& vis,
                             const CalibrationMonitor::State& cal, const OcclusionMap& occ, double fps,
                             double target_fps, double latency_ms, double detector_ms, bool calibrated) {
  HealthReport h;
  h.stability = stab;
  h.visibility = vis;
  h.calibration = cal;
  h.coverage = occ.coverage();
  h.occluded_segments = occ.segments();
  h.fps = fps;
  h.latency_ms = latency_ms;
  h.detector_ms = detector_ms;
  h.calibrated = calibrated;
  auto raise = [&](Level l, const std::string& why) {
    h.level = std::max(h.level, l);
    h.reasons.push_back(why);
  };
  double conf = 1.0;
  if (!calibrated) {
    raise(Level::Degraded, "camera not calibrated yet: 3D positions unavailable");
    conf *= 0.3;
  }
  if (fps < 0.3 * target_fps) {
    raise(Level::Fault, "processing rate " + fmt(fps, 1) + " Hz far below target");
    conf *= 0.4;
  } else if (fps < 0.7 * target_fps) {
    raise(Level::Degraded, "processing rate " + fmt(fps, 1) + " Hz below target");
    conf *= 0.85;
  }
  if (stab.status == "lost") {
    raise(Level::Fault, "camera view lost or obstructed");
    conf *= 0.2;
  } else if (stab.status == "moved") {
    raise(Level::Fault, "camera moved " + fmt(stab.drift_deg) + " deg: recalibration required");
    conf *= 0.3;
  } else if (stab.status == "shaking") {
    raise(Level::Degraded, "camera shaking: jitter " + fmt(stab.jitter_px, 1) + " px (compensated)");
    conf *= 0.85;
  } else if (stab.status == "drifted") {
    raise(Level::Degraded, "camera rotated " + fmt(stab.drift_deg) + " deg since calibration (compensated)");
    conf *= 0.9;
  }
  if (vis.frozen) {
    raise(Level::Fault, "video stream frozen");
  } else if (vis.score < 0.4) {
    std::string c;
    for (const auto& s : vis.conditions) c += (c.empty() ? "" : ", ") + s;
    raise(Level::Fault, "visibility too poor (" + c + ")");
  } else if (vis.score < 0.75) {
    std::string c;
    for (const auto& s : vis.conditions) c += (c.empty() ? "" : ", ") + s;
    raise(Level::Degraded, "reduced visibility (" + c + "), effective range " + fmt(vis.effective_range, 0) + " m");
  }
  conf *= vis.score;
  if (cal.status == "invalid") {
    raise(Level::Fault, "calibration invalid");
  } else if (cal.status == "suspect") {
    raise(Level::Degraded, "calibration suspect");
  }
  for (const auto& r : cal.reasons) h.reasons.push_back("calibration: " + r);
  conf *= cal.score;
  for (const auto& z : h.coverage) {
    const double blind = z.static_occluded + z.unobservable;
    if (blind > 0.3) raise(Level::Degraded, "zone " + z.zone + ": " + fmt(blind * 100, 0) + "% permanently not observable");
    if (z.dynamic_occluded > 0.25)
      h.reasons.push_back("zone " + z.zone + ": " + fmt(z.dynamic_occluded * 100, 0) + "% hidden behind vehicles");
  }
  h.confidence = std::clamp(conf, 0.0, 1.0);
  return h;
}

}  // namespace jp
