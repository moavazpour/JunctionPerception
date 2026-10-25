#include "jp/calib/auto_calibrator.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <random>
#include <spdlog/spdlog.h>

namespace jp {
namespace {
constexpr double kDeg = M_PI / 180.0;

double iou(const Eigen::Vector4d& a, const Eigen::Vector4d& b) {
  const double x0 = std::max(a[0], b[0]), y0 = std::max(a[1], b[1]);
  const double x1 = std::min(a[2], b[2]), y1 = std::min(a[3], b[3]);
  const double in = std::max(0.0, x1 - x0) * std::max(0.0, y1 - y0);
  const double ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - in;
  return ua > 0 ? in / ua : 0.0;
}

double cauchy(double r, double s) { return std::log1p((r / s) * (r / s)); }

double median(std::vector<double> v) {
  if (v.empty()) return 0;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}
}  // namespace

AutoCalibrator::AutoCalibrator(int width, int height, CalibrationPriors priors, Config cfg)
    : w_(width), h_(height), pri_(std::move(priors)), cfg_(cfg) {}

void AutoCalibrator::add(double stamp, const std::vector<ImageDetection>& dets) {
  if (t0_ < 0) t0_ = stamp;
  t_last_ = stamp;
  std::vector<char> used(dets.size(), false);
  for (auto& tr : live_) {
    const ImageDetection& last = tr.det.back();
    int best = -1;
    double best_iou = 0.3;
    for (size_t j = 0; j < dets.size(); ++j) {
      if (used[j] || dets[j].cls != last.cls) continue;
      const double v = iou(last.box2d, dets[j].box2d);
      if (v > best_iou) {
        best_iou = v;
        best = static_cast<int>(j);
      }
    }
    if (best >= 0) {
      used[best] = true;
      tr.t.push_back(stamp);
      tr.kp.push_back(dets[best].kp);
      tr.det.push_back(dets[best]);
    }
  }
  for (size_t j = 0; j < dets.size(); ++j) {
    const auto& d = dets[j];
    if (used[j] || d.score < cfg_.min_score || !isVehicle(d.cls)) continue;
    const auto& b = d.box2d;
    if (b[0] < 3 || b[1] < 3 || b[2] > w_ - 3 || b[3] > h_ - 3) continue;  // truncated boxes are biased
    live_.push_back({{stamp}, {d.kp}, {d}});
    ++n_dets_;
  }
  for (auto it = live_.begin(); it != live_.end();) {
    if (stamp - it->t.back() > 0.5) {
      if (it->t.size() >= 10) done_.push_back(std::move(*it));
      it = live_.erase(it);
    } else {
      ++it;
    }
  }
  n_dets_ = 0;
  for (const auto& t : done_) n_dets_ += t.det.size();
  if (done_.size() > 600) done_.erase(done_.begin(), done_.begin() + 100);
}

bool AutoCalibrator::ready() const {
  return t_last_ - t0_ >= cfg_.min_duration_s && static_cast<int>(done_.size()) >= cfg_.min_tracks &&
         static_cast<int>(n_dets_) >= cfg_.min_detections;
}

double AutoCalibrator::progress() const {
  if (t0_ < 0) return 0;
  return std::min({(t_last_ - t0_) / cfg_.min_duration_s, static_cast<double>(done_.size()) / cfg_.min_tracks,
                   static_cast<double>(n_dets_) / cfg_.min_detections, 1.0});
}

std::optional<Eigen::Vector2d> AutoCalibrator::vanishingPoint(const std::vector<ImgTrack>& tracks, int width,
                                                              int height) {
  struct L {
    Eigen::Vector3d l;  // a u + b v + c = 0, (a,b) unit
    Eigen::Vector2d mid, dir;
    double w;
  };
  std::vector<L> lines;
  for (const auto& t : tracks) {
    if (t.kp.size() < 10) continue;
    const double disp = (t.kp.back() - t.kp.front()).norm();
    if (disp < 0.04 * width) continue;
    std::vector<cv::Point2f> pts;
    for (const auto& p : t.kp) pts.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
    cv::Vec4f fl;
    cv::fitLine(pts, fl, cv::DIST_HUBER, 0, 0.01, 0.01);
    const Eigen::Vector2d d(fl[0], fl[1]), p0(fl[2], fl[3]);
    // Reject curved tracks (turning vehicles): residual of the straight-line fit.
    double res = 0;
    for (const auto& p : t.kp) res += std::abs(d.x() * (p.y() - p0.y()) - d.y() * (p.x() - p0.x()));
    res /= t.kp.size();
    if (res > 0.01 * disp + 2.0) continue;
    const Eigen::Vector2d n(-d.y(), d.x());
    lines.push_back({{n.x(), n.y(), -n.dot(p0)}, p0, d, disp});
  }
  if (lines.size() < 4) return std::nullopt;
  std::mt19937 rng(7);
  std::uniform_int_distribution<size_t> pick(0, lines.size() - 1);
  auto angErr = [](const L& l, const Eigen::Vector2d& vp) {
    Eigen::Vector2d to = vp - l.mid;
    if (to.norm() < 1e-6) return 0.0;
    to.normalize();
    return std::acos(std::min(1.0, std::abs(to.dot(l.dir))));
  };
  Eigen::Vector2d best_vp;
  double best_score = -1;
  for (int it = 0; it < 2000; ++it) {
    const auto& a = lines[pick(rng)];
    const auto& b = lines[pick(rng)];
    Eigen::Vector3d x = a.l.cross(b.l);
    if (std::abs(x.z()) < 1e-9) continue;
    const Eigen::Vector2d vp = x.head<2>() / x.z();
    if (vp.norm() > 50.0 * width) continue;
    double score = 0;
    for (const auto& l : lines)
      if (angErr(l, vp) < 2.0 * kDeg) score += l.w;
    if (score > best_score) {
      best_score = score;
      best_vp = vp;
    }
  }
  if (best_score < 0) return std::nullopt;
  // Least-squares refinement on inliers: minimise sum w (a u + b v + c)^2.
  for (int iter = 0; iter < 3; ++iter) {
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    Eigen::Vector2d bb = Eigen::Vector2d::Zero();
    int n = 0;
    for (const auto& l : lines) {
      if (angErr(l, best_vp) > 2.0 * kDeg) continue;
      const Eigen::Vector2d ab = l.l.head<2>();
      A += l.w * ab * ab.transpose();
      bb -= l.w * ab * l.l.z();
      ++n;
    }
    if (n < 3) break;
    best_vp = A.ldlt().solve(bb);
  }
  (void)height;
  return best_vp;
}

CameraModel AutoCalibrator::build(const Params& p, const Eigen::Vector2d& vp, double bearing) const {
  const double f = pri_.focal_px ? *pri_.focal_px : std::exp(p.logf);
  Eigen::Matrix3d K;
  K << f, 0, (w_ - 1) / 2.0, 0, f, (h_ - 1) / 2.0, 0, 0, 1;
  const Eigen::Vector3d d1 = (K.inverse() * Eigen::Vector3d(vp.x(), vp.y(), 1)).normalized();
  const Eigen::Vector3d up(0, -1, 0);
  const Eigen::Vector3d e1 = (up - up.dot(d1) * d1).normalized();
  const Eigen::Vector3d e2 = d1.cross(e1);
  const Eigen::Vector3d n = std::cos(p.psi) * e1 + std::sin(p.psi) * e2;
  CameraModel cam(w_, h_, K, Eigen::Vector4d(n.x(), n.y(), n.z(), p.height), bearing);
  cam.anchor = pri_.anchor;
  return cam;
}

double AutoCalibrator::cost(const Params& p, const Eigen::Vector2d& vp, double* box_res, double* speed_cv) const {
  const CameraModel cam = build(p, vp);
  // Box consistency.
  double box = 0;
  int nb = 0;
  std::vector<double> rels;
  const size_t stride = std::max<size_t>(1, n_dets_ / 1500);
  size_t k = 0;
  for (const auto& t : done_) {
    for (const auto& d : t.det) {
      if (k++ % stride) continue;
      const double bh = d.box2d[3] - d.box2d[1];
      if (bh < 15) continue;
      auto det3 = cam.lift(d);
      double r = 1.0;
      if (det3) {
        auto px = cam.projectBox(det3->bottom, det3->yaw, det3->dims);
        if (px) {
          double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
          for (const auto& q : *px) {
            x0 = std::min(x0, q.x());
            y0 = std::min(y0, q.y());
            x1 = std::max(x1, q.x());
            y1 = std::max(y1, q.y());
          }
          const double rt = (y0 - d.box2d[1]) / bh, rl = (x0 - d.box2d[0]) / bh, rr = (x1 - d.box2d[2]) / bh;
          box += cauchy(rt, 0.1) + 0.5 * cauchy(rl, 0.1) + 0.5 * cauchy(rr, 0.1);
          rels.push_back(std::sqrt(rt * rt + 0.25 * (rl * rl + rr * rr)));
          ++nb;
          continue;
        }
      }
      box += cauchy(r, 0.1) * 2;
      rels.push_back(r);
      ++nb;
    }
  }
  box /= std::max(nb, 1);
  // Speed consistency: far half vs. near half of every long track.
  double spd = 0;
  int ns = 0;
  std::vector<double> cvs;
  for (const auto& t : done_) {
    std::vector<Eigen::Vector2d> g;
    std::vector<double> ts;
    for (size_t i = 0; i < t.kp.size(); ++i) {
      auto q = cam.pixelToGround(t.kp[i], t.det[i].dz);
      if (!q) continue;
      g.push_back(q->head<2>());
      ts.push_back(t.t[i]);
    }
    if (g.size() < 10 || ts.back() - ts.front() < 1.0) continue;
    const size_t m = g.size() / 2;
    const double v1 = (g[m] - g[0]).norm() / std::max(ts[m] - ts[0], 1e-3);
    const double v2 = (g.back() - g[m]).norm() / std::max(ts.back() - ts[m], 1e-3);
    if (v1 + v2 < 4.0 || std::abs(g.back().norm() - g[0].norm()) < 10.0) continue;  // stationary or no range span
    const double r = std::log(std::max(v2, 0.1) / std::max(v1, 0.1));
    spd += cauchy(r, 0.08);
    cvs.push_back(std::abs(r));
    ++ns;
  }
  spd /= std::max(ns, 1);
  const double roll = cam.roll();
  const double prior = 0.02 * std::pow((p.height - pri_.height_m) / pri_.height_sigma, 2) +
                       0.02 * std::pow(roll / (pri_.roll_sigma_deg * kDeg), 2);
  if (box_res) *box_res = median(rels);
  if (speed_cv) *speed_cv = median(cvs);
  return box + 0.7 * spd + prior;
}

CalibrationResult AutoCalibrator::solve() const {
  CalibrationResult res;
  auto vp = vanishingPoint(done_, w_, h_);
  if (!vp) {
    res.message = "no dominant road direction found in vehicle trajectories";
    return res;
  }
  res.vanishing_point = *vp;
  res.tracks_used = static_cast<int>(done_.size());
  res.detections_used = static_cast<int>(n_dets_);

  // Coarse grid over focal length and roll-about-road, line search over height.
  Params best{std::log(1.0 * w_), 0.0, pri_.height_m};
  double best_c = 1e18;
  const double lf0 = std::log(pri_.focal_min_rel * w_), lf1 = std::log(pri_.focal_max_rel * w_);
  const int nf = pri_.focal_px ? 1 : 32;
  for (int i = 0; i < nf; ++i) {
    const double lf = pri_.focal_px ? std::log(*pri_.focal_px) : lf0 + (lf1 - lf0) * i / (nf - 1);
    for (double psi = -6 * kDeg; psi <= 6 * kDeg + 1e-9; psi += 2 * kDeg) {
      for (double h = std::max(2.0, pri_.height_m - 3 * pri_.height_sigma); h <= pri_.height_m + 3 * pri_.height_sigma;
           h += 0.75) {
        const Params p{lf, psi, h};
        const double c = cost(p, *vp);
        if (c < best_c) {
          best_c = c;
          best = p;
        }
      }
    }
  }
  // Nelder-Mead refinement.
  std::array<Params, 4> s = {best, Params{best.logf + 0.05, best.psi, best.height},
                             Params{best.logf, best.psi + 1 * kDeg, best.height},
                             Params{best.logf, best.psi, best.height + 0.4}};
  auto vec = [](const Params& p) { return Eigen::Vector3d(p.logf, p.psi, p.height); };
  auto par = [](const Eigen::Vector3d& v) { return Params{v[0], v[1], v[2]}; };
  std::array<double, 4> c;
  for (int i = 0; i < 4; ++i) c[i] = cost(s[i], *vp);
  for (int it = 0; it < 150; ++it) {
    std::array<int, 4> o = {0, 1, 2, 3};
    std::sort(o.begin(), o.end(), [&](int a, int b) { return c[a] < c[b]; });
    std::array<Params, 4> s2;
    std::array<double, 4> c2;
    for (int i = 0; i < 4; ++i) {
      s2[i] = s[o[i]];
      c2[i] = c[o[i]];
    }
    s = s2;
    c = c2;
    if (std::abs(c[3] - c[0]) < 1e-7) break;
    Eigen::Vector3d centroid = (vec(s[0]) + vec(s[1]) + vec(s[2])) / 3.0;
    const Eigen::Vector3d xr = centroid + (centroid - vec(s[3]));
    const double cr = cost(par(xr), *vp);
    if (cr < c[0]) {
      const Eigen::Vector3d xe = centroid + 2.0 * (centroid - vec(s[3]));
      const double ce = cost(par(xe), *vp);
      if (ce < cr) {
        s[3] = par(xe);
        c[3] = ce;
      } else {
        s[3] = par(xr);
        c[3] = cr;
      }
    } else if (cr < c[2]) {
      s[3] = par(xr);
      c[3] = cr;
    } else {
      const Eigen::Vector3d xc = centroid + 0.5 * (vec(s[3]) - centroid);
      const double cc = cost(par(xc), *vp);
      if (cc < c[3]) {
        s[3] = par(xc);
        c[3] = cc;
      } else {
        for (int i = 1; i < 4; ++i) {
          s[i] = par(vec(s[0]) + 0.5 * (vec(s[i]) - vec(s[0])));
          c[i] = cost(s[i], *vp);
        }
      }
    }
  }
  best = s[0];
  double box_res = 0, speed_cv = 0;
  cost(best, *vp, &box_res, &speed_cv);
  res.box_residual = box_res;
  res.speed_cv = speed_cv;

  // Bearing: align the vanishing direction with the road bearing from the map.
  CameraModel cam0 = build(best, *vp, 0.0);
  const Eigen::Vector3d d1 = (cam0.K().inverse() * Eigen::Vector3d(vp->x(), vp->y(), 1)).normalized();
  Eigen::Vector3d g = cam0.R_ground_cam() * d1;  // bearing 0: camera forward = north = +y
  if (g.y() < 0) g = -g;
  const double phi = std::atan2(g.x(), g.y()) / kDeg;  // vanishing direction relative to camera forward
  double bearing = pri_.camera_bearing_deg;
  if (pri_.road_bearing_deg) {
    double rb = *pri_.road_bearing_deg;
    auto angDiff = [](double a, double b) { return std::abs(std::remainder(a - b, 360.0)); };
    const double expected = pri_.camera_bearing_deg + phi;
    if (angDiff(rb + 180.0, expected) < angDiff(rb, expected)) rb += 180.0;
    bearing = std::fmod(rb - phi + 720.0, 360.0);
  }
  res.camera = build(best, *vp, bearing);
  res.ok = true;
  res.confidence = std::clamp(1.0 - 3.0 * std::max(0.0, box_res - 0.05) - 2.0 * speed_cv, 0.0, 1.0);
  res.message = "f=" + std::to_string(res.camera.K()(0, 0)) + " px, pitch=" + std::to_string(res.camera.pitch() / kDeg) +
                " deg, roll=" + std::to_string(res.camera.roll() / kDeg) + " deg, height=" +
                std::to_string(res.camera.cameraHeight()) + " m, bearing=" + std::to_string(bearing) + " deg";
  spdlog::info("Auto-calibration: {} (box residual {:.3f}, speed drift {:.3f}, confidence {:.2f})", res.message,
               box_res, speed_cv, res.confidence);
  return res;
}

}  // namespace jp
