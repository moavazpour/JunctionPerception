#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace jp {

// Oriented rectangle on the ground plane.
struct OrientedBox {
  Eigen::Vector2d c{0, 0};
  double yaw = 0, length = 0, width = 0;

  std::array<Eigen::Vector2d, 4> corners() const {
    const Eigen::Vector2d x(std::cos(yaw), std::sin(yaw)), y(-std::sin(yaw), std::cos(yaw));
    return {c + x * length / 2 + y * width / 2, c + x * length / 2 - y * width / 2,
            c - x * length / 2 - y * width / 2, c - x * length / 2 + y * width / 2};
  }
};

// Separating-axis test for two oriented rectangles.
inline bool overlaps(const OrientedBox& a, const OrientedBox& b) {
  const auto ca = a.corners(), cb = b.corners();
  const Eigen::Vector2d axes[4] = {{std::cos(a.yaw), std::sin(a.yaw)}, {-std::sin(a.yaw), std::cos(a.yaw)},
                                   {std::cos(b.yaw), std::sin(b.yaw)}, {-std::sin(b.yaw), std::cos(b.yaw)}};
  for (const auto& ax : axes) {
    double amin = 1e18, amax = -1e18, bmin = 1e18, bmax = -1e18;
    for (const auto& p : ca) {
      const double d = p.dot(ax);
      amin = std::min(amin, d);
      amax = std::max(amax, d);
    }
    for (const auto& p : cb) {
      const double d = p.dot(ax);
      bmin = std::min(bmin, d);
      bmax = std::max(bmax, d);
    }
    if (amax < bmin || bmax < amin) return false;
  }
  return true;
}

// Minimum distance between two oriented rectangles (0 when overlapping).
inline double distance(const OrientedBox& a, const OrientedBox& b) {
  if (overlaps(a, b)) return 0.0;
  auto segDist = [](const Eigen::Vector2d& p, const Eigen::Vector2d& s0, const Eigen::Vector2d& s1) {
    const Eigen::Vector2d d = s1 - s0;
    const double t = std::clamp((p - s0).dot(d) / std::max(d.squaredNorm(), 1e-12), 0.0, 1.0);
    return (p - (s0 + t * d)).norm();
  };
  const auto ca = a.corners(), cb = b.corners();
  double best = 1e18;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      best = std::min(best, segDist(ca[i], cb[j], cb[(j + 1) % 4]));
      best = std::min(best, segDist(cb[i], ca[j], ca[(j + 1) % 4]));
    }
  }
  return best;
}

inline double wrapAngle(double a) {
  a = std::fmod(a + M_PI, 2 * M_PI);
  if (a < 0) a += 2 * M_PI;
  return a - M_PI;
}

// Polyline helpers used by lanes and zones.
struct Polyline {
  std::vector<Eigen::Vector2d> pts;

  double length() const {
    double s = 0;
    for (size_t i = 1; i < pts.size(); ++i) s += (pts[i] - pts[i - 1]).norm();
    return s;
  }

  // Project a point: returns arc length s, signed lateral offset (left positive) and tangent heading.
  struct Projection {
    double s = 0, lateral = 0, heading = 0, dist = 1e18;
  };
  Projection project(const Eigen::Vector2d& p) const {
    Projection best;
    double acc = 0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const Eigen::Vector2d a = pts[i - 1], d = pts[i] - a;
      const double L = d.norm();
      if (L < 1e-9) continue;
      const Eigen::Vector2d u = d / L;
      const double t = std::clamp((p - a).dot(u), 0.0, L);
      const Eigen::Vector2d q = a + u * t;
      const double dist = (p - q).norm();
      if (dist < best.dist) {
        const Eigen::Vector2d r = p - q;
        best = {acc + t, u.x() * r.y() - u.y() * r.x(), std::atan2(u.y(), u.x()), dist};
      }
      acc += L;
    }
    return best;
  }

  Eigen::Vector2d at(double s) const {
    double acc = 0;
    for (size_t i = 1; i < pts.size(); ++i) {
      const double L = (pts[i] - pts[i - 1]).norm();
      if (acc + L >= s || i + 1 == pts.size()) {
        const double t = L > 1e-9 ? std::clamp((s - acc) / L, 0.0, 1.0) : 0.0;
        return pts[i - 1] + t * (pts[i] - pts[i - 1]);
      }
      acc += L;
    }
    return pts.empty() ? Eigen::Vector2d::Zero() : pts.back();
  }
};

inline bool pointInPolygon(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& p) {
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i].y() > p.y()) != (poly[j].y() > p.y())) &&
        (p.x() < (poly[j].x() - poly[i].x()) * (p.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()))
      in = !in;
  }
  return in;
}

}  // namespace jp
