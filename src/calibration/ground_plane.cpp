#include "jp/calibration/ground_plane.hpp"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <random>
#include <stdexcept>

namespace jp::calibration {

namespace {

constexpr double kMinTripleArea = 1e-6;

Plane oriented(Eigen::Vector3d normal, const Eigen::Vector3d& point) {
  normal.normalize();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  return Plane{normal, -normal.dot(point)};
}

Plane least_squares_plane(const std::vector<Eigen::Vector3d>& points) {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& p : points) {
    mean += p;
  }
  mean /= static_cast<double>(points.size());
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (const auto& p : points) {
    scatter += (p - mean) * (p - mean).transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  return oriented(solver.eigenvectors().col(0), mean);
}

}  // namespace

double Plane::signed_distance(const Eigen::Vector3d& point) const {
  return normal.dot(point) + offset;
}

Eigen::Vector3d Plane::project(const Eigen::Vector3d& point) const {
  return point - (signed_distance(point) * normal);
}

PlaneFit fit_plane(const std::vector<Eigen::Vector3d>& points, const PlaneFitOptions& options) {
  if (points.size() < 3) {
    throw std::invalid_argument("a plane fit needs at least three points");
  }
  std::mt19937 rng(options.seed);
  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);

  const auto inliers_of = [&](const Plane& plane) {
    std::vector<Eigen::Vector3d> inliers;
    for (const auto& p : points) {
      if (std::abs(plane.signed_distance(p)) < options.inlier_threshold_m) {
        inliers.push_back(p);
      }
    }
    return inliers;
  };

  std::vector<Eigen::Vector3d> best_inliers;
  for (int i = 0; i < options.iterations; ++i) {
    const auto& a = points[pick(rng)];
    const auto& b = points[pick(rng)];
    const auto& c = points[pick(rng)];
    const Eigen::Vector3d normal = (b - a).cross(c - a);
    if (normal.norm() < kMinTripleArea) {
      continue;
    }
    auto inliers = inliers_of(oriented(normal, a));
    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
    }
  }
  if (best_inliers.size() < 3) {
    throw std::runtime_error("no plane found among the points");
  }

  PlaneFit fit;
  fit.plane = least_squares_plane(best_inliers);
  const auto refined_inliers = inliers_of(fit.plane);
  fit.inliers = refined_inliers.size();
  double sum_sq = 0.0;
  for (const auto& p : refined_inliers) {
    sum_sq += fit.plane.signed_distance(p) * fit.plane.signed_distance(p);
  }
  fit.inlier_rms_m =
      refined_inliers.empty() ? 0.0 : std::sqrt(sum_sq / static_cast<double>(fit.inliers));
  return fit;
}

}  // namespace jp::calibration
