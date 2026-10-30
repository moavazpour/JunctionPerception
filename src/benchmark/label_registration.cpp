#include "jp/benchmark/label_registration.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "jp/core/assignment.hpp"

namespace jp::benchmark {

namespace {

using FramePairs = std::vector<std::pair<std::vector<Cuboid>, std::vector<Cuboid>>>;

constexpr double kForbiddenCost = 1e9;

double bev_distance(const Cuboid& a, const Cuboid& b) {
  return (a.center.head<2>() - b.center.head<2>()).norm();
}

std::vector<std::size_t> sample_indices(std::size_t size, std::size_t count) {
  std::vector<std::size_t> indices;
  if (size == 0) {
    return indices;
  }
  const std::size_t n = std::min(size, std::max<std::size_t>(count, 1));
  indices.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    indices.push_back((i * size) / n);
  }
  return indices;
}

std::vector<Cuboid> transformed(const std::vector<Cuboid>& boxes, const PlanarPose& pose) {
  std::vector<Cuboid> out;
  out.reserve(boxes.size());
  for (const auto& box : boxes) {
    out.push_back(pose.apply(box));
  }
  return out;
}

// Number of north boxes that have a south box of the same category within the gate.
std::size_t agreement(const FramePairs& frames, const std::vector<std::size_t>& indices,
                      const PlanarPose& pose, double gate_m) {
  std::size_t count = 0;
  for (const std::size_t i : indices) {
    const auto& [south, north] = frames[i];
    for (const auto& n : north) {
      const Cuboid moved = pose.apply(n);
      count += static_cast<std::size_t>(std::ranges::any_of(south, [&](const Cuboid& s) {
        return s.category == moved.category && bev_distance(s, moved) < gate_m;
      }));
    }
  }
  return count;
}

PlanarPose best_hypothesis(const FramePairs& frames, const RegistrationOptions& options) {
  const auto hypothesis_indices = sample_indices(frames.size(), options.hypothesis_frames);
  const auto scoring_indices = sample_indices(frames.size(), options.scoring_frames);
  PlanarPose best;
  std::size_t best_score = 0;
  for (const std::size_t i : hypothesis_indices) {
    const auto& [south, north] = frames[i];
    for (const auto& s : south) {
      if (s.size.x() < options.min_heading_length_m) {
        continue;
      }
      for (const auto& n : north) {
        if (n.category != s.category || n.size.x() < options.min_heading_length_m) {
          continue;
        }
        // Box headings are ambiguous by half a turn, so both orientations are hypotheses.
        for (const double flip : {0.0, std::numbers::pi}) {
          PlanarPose pose;
          pose.yaw_rad = s.yaw() - n.yaw() + flip;
          pose.translation =
              s.center - (Eigen::AngleAxisd(pose.yaw_rad, Eigen::Vector3d::UnitZ()) * n.center);
          const std::size_t score =
              agreement(frames, scoring_indices, pose, options.hypothesis_gate_m);
          if (score > best_score) {
            best_score = score;
            best = pose;
          }
        }
      }
    }
  }
  if (best_score == 0) {
    throw std::runtime_error("no consistent transform between the label sets found");
  }
  return best;
}

struct MatchedPair {
  Eigen::Vector3d south;
  Eigen::Vector3d north;  // in the north LiDAR frame
};

std::vector<MatchedPair> all_matches(const FramePairs& frames, const PlanarPose& pose,
                                     double gate_m) {
  std::vector<MatchedPair> pairs;
  for (const auto& [south, north] : frames) {
    const auto moved = transformed(north, pose);
    for (const auto& [si, ni] : match_boxes(south, moved, gate_m)) {
      pairs.push_back({south[si].center, north[ni].center});
    }
  }
  return pairs;
}

// Least-squares yaw and translation mapping north centres onto south centres.
PlanarPose fit_pose(const std::vector<MatchedPair>& pairs) {
  Eigen::Vector3d mean_s = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_n = Eigen::Vector3d::Zero();
  for (const auto& p : pairs) {
    mean_s += p.south;
    mean_n += p.north;
  }
  mean_s /= static_cast<double>(pairs.size());
  mean_n /= static_cast<double>(pairs.size());
  double cross = 0.0;
  double dot = 0.0;
  for (const auto& p : pairs) {
    const Eigen::Vector2d s = (p.south - mean_s).head<2>();
    const Eigen::Vector2d n = (p.north - mean_n).head<2>();
    cross += (n.x() * s.y()) - (n.y() * s.x());
    dot += (n.x() * s.x()) + (n.y() * s.y());
  }
  PlanarPose pose;
  pose.yaw_rad = std::atan2(cross, dot);
  pose.translation = mean_s - (Eigen::AngleAxisd(pose.yaw_rad, Eigen::Vector3d::UnitZ()) * mean_n);
  return pose;
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  const auto k = static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1));
  std::ranges::nth_element(values, values.begin() + static_cast<std::ptrdiff_t>(k));
  return values[k];
}

}  // namespace

Eigen::Isometry3d PlanarPose::isometry() const {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  iso.translation() = translation;
  return iso;
}

Cuboid PlanarPose::apply(const Cuboid& box) const {
  Cuboid out = box;
  const Eigen::Quaterniond rotation(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
  out.center = (rotation * box.center) + translation;
  out.rotation = (rotation * box.rotation).normalized();
  return out;
}

std::vector<std::pair<std::size_t, std::size_t>> match_boxes(const std::vector<Cuboid>& south,
                                                             const std::vector<Cuboid>& north,
                                                             double gate_m) {
  std::vector<std::pair<std::size_t, std::size_t>> matches;
  if (south.empty() || north.empty()) {
    return matches;
  }
  Eigen::MatrixXd cost(static_cast<Eigen::Index>(south.size()),
                       static_cast<Eigen::Index>(north.size()));
  for (std::size_t i = 0; i < south.size(); ++i) {
    for (std::size_t j = 0; j < north.size(); ++j) {
      const double d = bev_distance(south[i], north[j]);
      const bool allowed = south[i].category == north[j].category && d < gate_m;
      cost(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
          allowed ? d : kForbiddenCost;
    }
  }
  const auto assignment = core::solve_assignment(cost);
  for (std::size_t i = 0; i < assignment.size(); ++i) {
    const Eigen::Index j = assignment[i];
    if (j >= 0 && cost(static_cast<Eigen::Index>(i), j) < kForbiddenCost) {
      matches.emplace_back(i, static_cast<std::size_t>(j));
    }
  }
  return matches;
}

RegistrationResult register_label_sets(const FramePairs& frames,
                                       const RegistrationOptions& options) {
  PlanarPose pose = best_hypothesis(frames, options);
  for (const double gate : options.refine_gates_m) {
    const auto pairs = all_matches(frames, pose, gate);
    if (pairs.size() < 3) {
      throw std::runtime_error("too few matched boxes to refine the label registration");
    }
    pose = fit_pose(pairs);
  }

  RegistrationResult result;
  result.south_from_north = pose;
  std::vector<double> bev_errors;
  std::vector<double> height_errors;
  for (const auto& [south, north] : frames) {
    const auto moved = transformed(north, pose);
    for (const auto& [si, ni] : match_boxes(south, moved, options.report_gate_m)) {
      bev_errors.push_back(bev_distance(south[si], moved[ni]));
      height_errors.push_back(std::abs(south[si].center.z() - moved[ni].center.z()));
    }
  }
  result.matched_pairs = bev_errors.size();
  result.median_bev_error_m = percentile(bev_errors, 0.5);
  result.p90_bev_error_m = percentile(bev_errors, 0.9);
  result.median_height_error_m = percentile(height_errors, 0.5);
  return result;
}

}  // namespace jp::benchmark
