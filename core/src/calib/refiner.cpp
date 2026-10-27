#include "jp/calib/refiner.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>

namespace jp {
namespace {
constexpr double kDeg = M_PI / 180.0;

double wrapHalf(double a) {  // to (-pi/2, pi/2]: directions without sign
  a = std::remainder(a, M_PI);
  return a;
}

// Plain Nelder-Mead (small, bounded problems).
Eigen::VectorXd nelderMead(const std::function<double(const Eigen::VectorXd&)>& f, Eigen::VectorXd x0,
                           const Eigen::VectorXd& step, int iters) {
  const int n = static_cast<int>(x0.size());
  std::vector<Eigen::VectorXd> s(n + 1, x0);
  std::vector<double> v(n + 1);
  for (int i = 0; i < n; ++i) s[i + 1][i] += step[i];
  for (int i = 0; i <= n; ++i) v[i] = f(s[i]);
  for (int it = 0; it < iters; ++it) {
    std::vector<int> o(n + 1);
    for (int i = 0; i <= n; ++i) o[i] = i;
    std::sort(o.begin(), o.end(), [&](int a, int b) { return v[a] < v[b]; });
    const int best = o[0], worst = o[n], second = o[n - 1];
    Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) c += s[o[i]];
    c /= n;
    const Eigen::VectorXd xr = c + (c - s[worst]);
    const double fr = f(xr);
    if (fr < v[best]) {
      const Eigen::VectorXd xe = c + 2.0 * (c - s[worst]);
      const double fe = f(xe);
      if (fe < fr) s[worst] = xe, v[worst] = fe;
      else s[worst] = xr, v[worst] = fr;
    } else if (fr < v[second]) {
      s[worst] = xr, v[worst] = fr;
    } else {
      const Eigen::VectorXd xc = c + 0.5 * (s[worst] - c);
      const double fc = f(xc);
      if (fc < v[worst]) {
        s[worst] = xc, v[worst] = fc;
      } else {
        for (int i = 1; i <= n; ++i) {
          s[o[i]] = s[best] + 0.5 * (s[o[i]] - s[best]);
          v[o[i]] = f(s[o[i]]);
        }
      }
    }
  }
  return s[std::min_element(v.begin(), v.end()) - v.begin()];
}
}  // namespace

void CalibrationRefiner::reset(const CameraModel& cam, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> points,
                               std::vector<Lane> lanes) {
  cam_ = cam;
  points_ = std::move(points);
  lanes_ = std::move(lanes);
  paths_.clear();
}

void CalibrationRefiner::add(std::uint32_t track, double stamp, const Eigen::Vector2d& px) {
  if (!cam_.valid()) return;
  auto& p = paths_[track];
  if (p.px.empty() || (p.px.back() - px).norm() > 2.0) p.px.push_back(px);
  p.last = stamp;
  if (paths_.size() > 600) {  // keep the newest paths
    auto oldest = std::min_element(paths_.begin(), paths_.end(), [](const auto& a, const auto& b) { return a.second.last < b.second.last; });
    paths_.erase(oldest);
  }
}

int CalibrationRefiner::usableTracks() const {
  int n = 0;
  for (const auto& [id, p] : paths_) {
    if (p.px.size() < 8) continue;
    const double ext = (p.px.back() - p.px.front()).norm();
    if (ext > 40.0) ++n;
  }
  return n;
}

double CalibrationRefiner::spread(const CameraModel& cam, int* used) const {
  struct Dir {
    double theta, w;
    Eigen::Vector2d mid;
  };
  std::vector<Dir> dirs;
  for (const auto& [id, p] : paths_) {
    if (p.px.size() < 8) continue;
    std::vector<Eigen::Vector2d> g;
    for (const auto& q : p.px)
      if (auto x = cam.pixelToGround(q)) g.push_back(x->head<2>());
    if (g.size() < 8) continue;
    const double len = (g.back() - g.front()).norm();
    if (len < cfg_.min_path_m || len > 400) continue;
    Eigen::Vector2d m = Eigen::Vector2d::Zero();
    for (const auto& x : g) m += x;
    m /= g.size();
    Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
    for (const auto& x : g) C += (x - m) * (x - m).transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C / g.size());
    // Straight paths only (turning vehicles say nothing about parallelism).
    if (std::sqrt(std::max(es.eigenvalues()[0], 0.0)) > 0.6) continue;
    const Eigen::Vector2d d = es.eigenvectors().col(1);
    dirs.push_back({std::atan2(d.y(), d.x()), len, m});
  }
  if (used) *used = static_cast<int>(dirs.size());
  if (dirs.size() < 5) return 1e3;
  std::vector<double> dev;
  std::vector<double> w;
  if (!lanes_.empty()) {
    for (const auto& d : dirs) {
      const Lane* best = nullptr;
      Polyline::Projection bp;
      for (const auto& l : lanes_) {
        const auto pr = l.centerline.project(d.mid);
        if (pr.dist < bp.dist) bp = pr, best = &l;
      }
      if (!best || bp.dist > best->width) continue;
      const double e = wrapHalf(d.theta - bp.heading);
      if (std::abs(e) > 25 * kDeg) continue;
      dev.push_back(e);
      w.push_back(d.w);
    }
  } else {
    // Dominant directions: up to two clusters of doubled angles (main road and cross road).
    std::vector<double> centres;
    auto mean = [&](const std::vector<int>& idx) {
      double s = 0, c = 0;
      for (int i : idx) s += dirs[i].w * std::sin(2 * dirs[i].theta), c += dirs[i].w * std::cos(2 * dirs[i].theta);
      return 0.5 * std::atan2(s, c);
    };
    std::vector<int> all(dirs.size());
    for (size_t i = 0; i < dirs.size(); ++i) all[i] = static_cast<int>(i);
    centres.push_back(mean(all));
    std::vector<int> far;
    for (int i : all)
      if (std::abs(wrapHalf(dirs[i].theta - centres[0])) > 30 * kDeg) far.push_back(i);
    if (far.size() > dirs.size() / 4) centres.push_back(mean(far));
    for (int round = 0; round < 3; ++round) {
      std::vector<std::vector<int>> mem(centres.size());
      for (int i : all) {
        int k = 0;
        for (size_t c = 1; c < centres.size(); ++c)
          if (std::abs(wrapHalf(dirs[i].theta - centres[c])) < std::abs(wrapHalf(dirs[i].theta - centres[k]))) k = static_cast<int>(c);
        if (std::abs(wrapHalf(dirs[i].theta - centres[k])) < 25 * kDeg) mem[k].push_back(i);
      }
      for (size_t c = 0; c < centres.size(); ++c)
        if (!mem[c].empty()) centres[c] = mean(mem[c]);
    }
    for (const auto& d : dirs) {
      double e = 1e9;
      for (double c : centres)
        if (std::abs(wrapHalf(d.theta - c)) < std::abs(e)) e = wrapHalf(d.theta - c);
      if (std::abs(e) > 25 * kDeg) continue;
      dev.push_back(e);
      w.push_back(d.w);
    }
  }
  if (dev.size() < 5) return 1e3;
  double s = 0, sw = 0;
  for (size_t i = 0; i < dev.size(); ++i) s += w[i] * dev[i] * dev[i], sw += w[i];
  return std::sqrt(s / sw) / kDeg;
}

double CalibrationRefiner::pointsRms(const CameraModel& cam) const {
  if (points_.empty()) return 0;
  double s = 0;
  for (const auto& [px, g] : points_) {
    const auto x = cam.pixelToGround(px);
    const double e = x ? (x->head<2>() - g).norm() : 50.0;
    s += e * e;
  }
  return std::sqrt(s / points_.size());
}

CameraModel CalibrationRefiner::apply(const Eigen::Vector4d& d) const {
  // Rotations in the camera frame (x right, y down, z forward): pitch about x, yaw about y, roll about z.
  const Eigen::Matrix3d R = (Eigen::AngleAxisd(d[0], Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(d[2], Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(d[1], Eigen::Vector3d::UnitZ())).toRotationMatrix();
  CameraModel r = cam_.rotated(R);
  Eigen::Matrix3d K = r.K();
  K(0, 0) *= std::exp(d[3]);
  K(1, 1) *= std::exp(d[3]);
  CameraModel out(r.width(), r.height(), K, r.planeCam(), r.bearingDeg(), r.distortion());
  out.anchor = cam_.anchor;
  return out;
}

std::optional<CalibrationRefiner::Proposal> CalibrationRefiner::solve() const {
  if (!cam_.valid()) return std::nullopt;
  int used = 0;
  const double s0 = spread(cam_, &used);
  if (s0 >= 1e3 || s0 < 0.3) return std::nullopt;  // too little data, or already consistent
  const double p0 = pointsRms(cam_);
  const double sp = std::max(p0, 0.3);
  const bool use_yaw = !lanes_.empty();
  const double rb = cfg_.max_rot_deg * kDeg, fb = std::log1p(cfg_.max_focal_rel);
  auto vec = [&](const Eigen::VectorXd& x) {
    return Eigen::Vector4d(x[0], x[1], use_yaw ? x[3] : 0.0, x[2]);
  };
  auto cost = [&](const Eigen::VectorXd& x) {
    const Eigen::Vector4d d = vec(x);
    double pen = 0;
    for (int k = 0; k < 3; ++k) pen += std::pow(std::max(0.0, std::abs(d[k]) - rb) / kDeg, 2) * 100;
    pen += std::pow(std::max(0.0, std::abs(d[3]) - fb) * 100, 2);
    const CameraModel c = apply(d);
    const double s = spread(c);
    const double p = pointsRms(c);
    return s * s + std::pow(p / sp, 2) * 2.0 + pen;
  };
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(use_yaw ? 4 : 3);
  Eigen::VectorXd step(x0.size());
  step << 0.6 * kDeg, 0.6 * kDeg, 0.04;
  if (use_yaw) step[3] = 0.6 * kDeg;
  const Eigen::VectorXd x = nelderMead(cost, x0, step, 250);
  const Eigen::Vector4d d = vec(x);
  Proposal pr;
  pr.camera = apply(d);
  pr.spread_before_deg = s0;
  pr.spread_after_deg = spread(pr.camera);
  pr.points_before_m = p0;
  pr.points_after_m = pointsRms(pr.camera);
  pr.d_pitch_deg = d[0] / kDeg;
  pr.d_roll_deg = d[1] / kDeg;
  pr.d_yaw_deg = d[2] / kDeg;
  pr.d_focal_rel = std::exp(d[3]) - 1;
  pr.tracks = used;
  const bool better = pr.spread_after_deg < (1 - cfg_.min_gain) * s0;
  const bool keeps_points = pr.points_after_m <= std::max(p0 * 1.25, p0 + 0.15);
  if (!better || !keeps_points) return std::nullopt;
  return pr;
}

}  // namespace jp
