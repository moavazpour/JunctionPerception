#include "jp/localization/validated_region.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "jp/localization/ground_projection.hpp"

namespace jp::localization {

namespace {

constexpr double kEpsilon = 1e-9;

double cross(const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return ((a.x() - o.x()) * (b.y() - o.y())) - ((a.y() - o.y()) * (b.x() - o.x()));
}

// Convex hull, counter-clockwise (Andrew's monotone chain).
std::vector<Eigen::Vector2d> convex_hull(std::vector<Eigen::Vector2d> points) {
  std::ranges::sort(points, [](const auto& a, const auto& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  if (points.size() < 3) {
    return points;
  }
  std::vector<Eigen::Vector2d> hull(2 * points.size());
  std::size_t k = 0;
  for (const auto& p : points) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0) {
      --k;
    }
    hull[k++] = p;
  }
  for (std::size_t i = points.size() - 1, lower = k + 1; i-- > 0;) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], points[i]) <= 0) {
      --k;
    }
    hull[k++] = points[i];
  }
  hull.resize(k - 1);
  return hull;
}

double segment_distance(const Eigen::Vector2d& p, const Eigen::Vector2d& a,
                        const Eigen::Vector2d& b) {
  const Eigen::Vector2d ab = b - a;
  const double length_sq = ab.squaredNorm();
  const double t = length_sq < kEpsilon ? 0.0 : std::clamp((p - a).dot(ab) / length_sq, 0.0, 1.0);
  return (a + (t * ab) - p).norm();
}

}  // namespace

ValidatedRegion::ValidatedRegion(const std::vector<Eigen::Vector2d>& control_points,
                                 const Eigen::Matrix3d& image_to_ground,
                                 Eigen::Vector2d camera_position, ValidatedRegionOptions options)
    : outline_(convex_hull(control_points)),
      image_to_ground_(image_to_ground),
      ground_to_image_(image_to_ground.inverse()),
      camera_position_(std::move(camera_position)),
      options_(options) {
  if (!control_points.empty()) {
    front_sign_ = (ground_to_image_ * control_points.front().homogeneous()).z() < 0.0 ? -1.0 : 1.0;
  }
}

double ValidatedRegion::distance_to_outline(const Eigen::Vector2d& point) const {
  if (outline_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (outline_.size() >= 3) {
    bool inside = true;
    for (std::size_t i = 0; i < outline_.size() && inside; ++i) {
      inside = cross(outline_[i], outline_[(i + 1) % outline_.size()], point) >= 0.0;
    }
    if (inside) {
      return 0.0;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < outline_.size(); ++i) {
    best =
        std::min(best, segment_distance(point, outline_[i], outline_[(i + 1) % outline_.size()]));
  }
  return best;
}

double ValidatedRegion::ground_m_per_px(const Eigen::Vector2d& point) const {
  const Eigen::Vector3d image = ground_to_image_ * point.homogeneous();
  const Eigen::Vector2d ray = point - camera_position_;
  if (front_sign_ * image.z() <= kEpsilon || ray.norm() < kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  const auto jacobian = ground_jacobian(image.hnormalized(), image_to_ground_);
  if (!jacobian) {
    return std::numeric_limits<double>::infinity();
  }
  // Ground motion along the ray for a unit image step in the direction that moves it most.
  return (ray.normalized().transpose() * (*jacobian)).norm();
}

bool ValidatedRegion::contains(const Eigen::Vector2d& point) const {
  return distance_to_outline(point) <= options_.margin_m &&
         ground_m_per_px(point) <= options_.max_ground_m_per_px;
}

}  // namespace jp::localization
