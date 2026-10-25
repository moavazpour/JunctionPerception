#include "jp/tracking/tracker.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

#include "jp/geometry/bev.hpp"

namespace jp {

// ---------------------------------------------------------------- CTRA EKF

CtraEkf::Vec CtraEkf::f(const Vec& s, double dt) {
  Vec o = s;
  const double x = s[0], y = s[1], th = s[2], v = s[3], w = s[4], a = s[5];
  if (std::abs(w) > 1e-3) {
    const double th1 = th + w * dt;
    const double w2 = w * w;
    o[0] = x + ((v * w + a * w * dt) * std::sin(th1) + a * std::cos(th1) - v * w * std::sin(th) - a * std::cos(th)) / w2;
    o[1] = y + ((-v * w - a * w * dt) * std::cos(th1) + a * std::sin(th1) + v * w * std::cos(th) - a * std::sin(th)) / w2;
  } else {
    const double d = v * dt + 0.5 * a * dt * dt;
    o[0] = x + d * std::cos(th);
    o[1] = y + d * std::sin(th);
  }
  o[2] = th + w * dt;
  o[3] = v + a * dt;
  return o;
}

void CtraEkf::init(const Eigen::Vector2d& p, double yaw, const Eigen::Matrix2d& pos_cov, double yaw_sigma) {
  x << p.x(), p.y(), yaw, 0, 0, 0;
  P.setZero();
  P.topLeftCorner<2, 2>() = pos_cov;
  P(2, 2) = yaw_sigma * yaw_sigma;
  P(3, 3) = 8.0 * 8.0;
  P(4, 4) = 0.3 * 0.3;
  P(5, 5) = 2.0 * 2.0;
}

void CtraEkf::predict(double dt, const Noise& q) {
  if (dt <= 0) return;
  Mat F;
  const Vec fx = f(x, dt);
  for (int k = 0; k < 6; ++k) {
    Vec xp = x;
    const double h = (k == 3 || k == 5) ? 1e-3 : 1e-5;
    xp[k] += h;
    F.col(k) = (f(xp, dt) - fx) / h;
  }
  // Process noise from jerk and yaw acceleration (piecewise-constant white noise model).
  const double dt2 = dt * dt, dt3 = dt2 * dt;
  Mat Q = Mat::Zero();
  const double qa = q.jerk * q.jerk, qw = q.yaw_acc * q.yaw_acc;
  Q(5, 5) = qa * dt;
  Q(3, 3) = qa * dt3 / 3;
  Q(3, 5) = Q(5, 3) = qa * dt2 / 2;
  Q(4, 4) = qw * dt;
  Q(2, 2) = qw * dt3 / 3;
  Q(2, 4) = Q(4, 2) = qw * dt2 / 2;
  Q(0, 0) += q.pos * q.pos * dt;
  Q(1, 1) += q.pos * q.pos * dt;
  // Longitudinal velocity uncertainty also spreads into position along the heading.
  const Eigen::Vector2d u(std::cos(x[2]), std::sin(x[2]));
  Q.topLeftCorner<2, 2>() += (qa * dt3 * dt2 / 20.0) * u * u.transpose();
  x = fx;
  P = F * P * F.transpose() + Q;
  normalize();
}

double CtraEkf::mahalanobis2(const Eigen::Vector2d& p, const Eigen::Matrix2d& pos_cov) const {
  const Eigen::Matrix2d S = P.topLeftCorner<2, 2>() + pos_cov;
  const Eigen::Vector2d r = p - x.head<2>();
  return r.dot(S.ldlt().solve(r));
}

void CtraEkf::update(const Eigen::Vector2d& p, const Eigen::Matrix2d& pos_cov, double yaw, double yaw_sigma,
                     bool use_yaw) {
  if (use_yaw) {
    // Fold the measured heading to the hypothesis closest to the state (handles front/back ambiguity).
    double dy = wrapAngle(yaw - x[2]);
    if (std::abs(dy) > M_PI / 2) dy = wrapAngle(dy + M_PI);
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = H(1, 1) = H(2, 2) = 1;
    Eigen::Vector3d r(p.x() - x[0], p.y() - x[1], dy);
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R.topLeftCorner<2, 2>() = pos_cov;
    R(2, 2) = yaw_sigma * yaw_sigma;
    const Eigen::Matrix3d S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();
    x += K * r;
    P = (Mat::Identity() - K * H) * P;
  } else {
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    H(0, 0) = H(1, 1) = 1;
    const Eigen::Vector2d r = p - x.head<2>();
    const Eigen::Matrix2d S = H * P * H.transpose() + pos_cov;
    const Eigen::Matrix<double, 6, 2> K = P * H.transpose() * S.inverse();
    x += K * r;
    P = (Mat::Identity() - K * H) * P;
  }
  P = 0.5 * (P + P.transpose());
  normalize();
}

void CtraEkf::normalize() {
  // Keep speed non-negative: a negative speed means the heading is reversed.
  if (x[3] < -0.3) {
    x[2] += M_PI;
    x[3] = -x[3];
    x[5] = -x[5];
  }
  x[2] = wrapAngle(x[2]);
}

// ---------------------------------------------------------------- Hungarian

std::vector<int> hungarian(const std::vector<std::vector<double>>& cost, double inf_cost) {
  const int n = static_cast<int>(cost.size());
  if (n == 0) return {};
  const int m = static_cast<int>(cost[0].size());
  if (m == 0) return std::vector<int>(n, -1);
  // Pad to square.
  const int N = std::max(n, m);
  std::vector<std::vector<double>> a(N + 1, std::vector<double>(N + 1, inf_cost));
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < m; ++j) a[i + 1][j + 1] = std::min(cost[i][j], inf_cost);
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> u(N + 1), v(N + 1);
  std::vector<int> p(N + 1), way(N + 1);
  for (int i = 1; i <= N; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(N + 1, INF);
    std::vector<char> used(N + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = INF;
      int j1 = 0;
      for (int j = 1; j <= N; ++j) {
        if (used[j]) continue;
        const double cur = a[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= N; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }
  std::vector<int> res(n, -1);
  for (int j = 1; j <= N; ++j) {
    const int i = p[j] - 1;
    if (i >= 0 && i < n && j - 1 < m && cost[i][j - 1] < inf_cost) res[i] = j - 1;
  }
  return res;
}

// ---------------------------------------------------------------- Tracker

CtraEkf::Noise Tracker::noiseFor(ObjectClass c) {
  switch (c) {
    case ObjectClass::Pedestrian: return {1.5, 1.5, 0.3};
    case ObjectClass::Bicycle:
    case ObjectClass::EScooter: return {2.0, 1.0, 0.2};
    case ObjectClass::Bus:
    case ObjectClass::Truck:
    case ObjectClass::Trailer: return {1.5, 0.3, 0.05};
    default: return {2.5, 0.6, 0.08};
  }
}

namespace {
bool compatible(ObjectClass a, ObjectClass b) {
  if (a == b) return true;
  auto group = [](ObjectClass c) {
    switch (c) {
      case ObjectClass::Car:
      case ObjectClass::Bus:
      case ObjectClass::Truck:
      case ObjectClass::Trailer: return 0;
      case ObjectClass::Pedestrian: return 1;
      default: return 2;
    }
  };
  return group(a) == group(b);
}

double yawSigmaFor(const Detection3D& d) {
  // Orientation of small or far objects is less reliable.
  const double box_h = d.img.box2d[3] - d.img.box2d[1];
  return std::clamp(0.15 + 10.0 / std::max(box_h, 1.0), 0.15, 1.2);
}
}  // namespace

void Tracker::applyMeasurement(Track& t, const Detection3D& d, double stamp) {
  const bool vru = isVulnerable(d.img.cls);
  // Pedestrian orientation is weakly observable; rely on motion instead.
  const bool use_yaw = !(d.img.cls == ObjectClass::Pedestrian);
  t.ekf.update(d.bottom.head<2>(), d.pos_cov, d.yaw, yawSigmaFor(d), use_yaw);
  const double w = std::clamp(static_cast<double>(d.img.score), 0.05, 1.0);
  t.class_votes[static_cast<size_t>(d.img.cls)] += static_cast<float>(w);
  const auto best = std::max_element(t.class_votes.begin(), t.class_votes.end());
  t.cls = static_cast<ObjectClass>(best - t.class_votes.begin());
  const double alpha = vru ? 0.2 : 0.1;
  t.dims = (1 - alpha) * t.dims + alpha * d.dims;
  t.hits++;
  t.misses = 0;
  t.last_update = stamp;
  t.score = 0.7f * t.score + 0.3f * d.img.score;
  t.last_box2d = d.img.box2d;
  t.last_range = d.range;
  if (t.state != TrackState::Confirmed && t.hits >= cfg_.confirm_hits) t.state = TrackState::Confirmed;
  if (t.state == TrackState::Coasting) t.state = TrackState::Confirmed;
}

void Tracker::associate(std::vector<int>& track_idx, const std::vector<int>& det_idx,
                        const std::vector<Detection3D>& dets, std::vector<int>& det_to_track) {
  if (track_idx.empty() || det_idx.empty()) return;
  constexpr double kInf = 1e6;
  std::vector<std::vector<double>> cost(track_idx.size(), std::vector<double>(det_idx.size(), kInf));
  for (size_t i = 0; i < track_idx.size(); ++i) {
    const Track& t = tracks_[track_idx[i]];
    for (size_t j = 0; j < det_idx.size(); ++j) {
      const Detection3D& d = dets[det_idx[j]];
      if (!compatible(t.cls, d.img.cls)) continue;
      const double jump = (d.bottom.head<2>() - t.position()).norm();
      const double max_jump = isVulnerable(t.cls) ? cfg_.max_jump_vru : cfg_.max_jump_vehicle;
      // Far objects carry larger range noise: scale the hard gate with the measurement covariance.
      const double gate = max_jump + 3.0 * std::sqrt(d.pos_cov.trace());
      if (jump > gate) continue;
      const double m2 = t.ekf.mahalanobis2(d.bottom.head<2>(), d.pos_cov);
      if (m2 > cfg_.gate_chi2) continue;
      cost[i][j] = m2 + (t.cls == d.img.cls ? 0.0 : 2.0);
    }
  }
  const auto assign = hungarian(cost, kInf);
  std::vector<int> remaining;
  for (size_t i = 0; i < assign.size(); ++i) {
    if (assign[i] >= 0) {
      det_to_track[det_idx[assign[i]]] = track_idx[i];
    } else {
      remaining.push_back(track_idx[i]);
    }
  }
  track_idx.swap(remaining);
}

const std::vector<Track>& Tracker::update(double stamp, const std::vector<Detection3D>& dets,
                                          const std::function<bool(const Eigen::Vector2d&)>& is_occluded) {
  const double dt = last_stamp_ < 0 ? 0.0 : std::clamp(stamp - last_stamp_, 0.0, 1.0);
  last_stamp_ = stamp;
  for (auto& t : tracks_) t.ekf.predict(dt, noiseFor(t.cls));

  std::vector<int> high, low;
  for (int j = 0; j < static_cast<int>(dets.size()); ++j) {
    if (dets[j].img.score >= cfg_.high_score) {
      high.push_back(j);
    } else if (dets[j].img.score >= cfg_.low_score) {
      low.push_back(j);
    }
  }
  std::vector<int> det_to_track(dets.size(), -1);
  // Stage 1: confirmed tracks with high-score detections, then tentative tracks.
  std::vector<int> confirmed, tentative;
  for (int i = 0; i < static_cast<int>(tracks_.size()); ++i)
    (tracks_[i].state == TrackState::Tentative ? tentative : confirmed).push_back(i);
  associate(confirmed, high, dets, det_to_track);
  std::vector<int> high_left;
  for (int j : high)
    if (det_to_track[j] < 0) high_left.push_back(j);
  associate(tentative, high_left, dets, det_to_track);
  // Stage 2 (ByteTrack): remaining confirmed tracks with low-score detections.
  associate(confirmed, low, dets, det_to_track);

  std::vector<char> matched(tracks_.size(), false);
  for (size_t j = 0; j < dets.size(); ++j) {
    if (det_to_track[j] >= 0) {
      applyMeasurement(tracks_[det_to_track[j]], dets[j], stamp);
      matched[det_to_track[j]] = true;
    }
  }
  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (matched[i]) continue;
    Track& t = tracks_[i];
    t.misses++;
    if (t.state == TrackState::Confirmed) t.state = TrackState::Coasting;
  }
  // Remove dead tracks.
  std::erase_if(tracks_, [&](const Track& t) {
    const double age = stamp - t.last_update;
    if (t.state == TrackState::Tentative) return t.misses > 1;
    const bool occl = is_occluded && is_occluded(t.position());
    return age > (occl ? cfg_.max_coast_occluded_s : cfg_.max_coast_s);
  });
  // Spawn new tracks from unmatched confident detections.
  for (size_t j = 0; j < dets.size(); ++j) {
    if (det_to_track[j] >= 0 || dets[j].img.score < cfg_.new_track_score) continue;
    const auto& d = dets[j];
    Track t;
    t.id = next_id_++;
    t.cls = d.img.cls;
    t.class_votes[static_cast<size_t>(d.img.cls)] = d.img.score;
    t.ekf.init(d.bottom.head<2>(), d.yaw, d.pos_cov, yawSigmaFor(d) * 1.5);
    t.dims = d.dims;
    t.hits = 1;
    t.first_seen = t.last_update = stamp;
    t.score = d.img.score;
    t.last_box2d = d.img.box2d;
    t.last_range = d.range;
    tracks_.push_back(t);
  }
  return tracks_;
}

}  // namespace jp
