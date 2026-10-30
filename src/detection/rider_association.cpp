#include "jp/detection/rider_association.hpp"

#include <Eigen/Core>
#include <algorithm>

#include "jp/core/assignment.hpp"

namespace jp::detection {

namespace {

constexpr double kForbiddenCost = 1e9;

// Share of the person box covered by the vehicle box, or 0 if the pair is not a plausible rider.
float rider_overlap(const cv::Rect2f& person, const cv::Rect2f& vehicle,
                    const RiderAssociationOptions& options) {
  const float person_area = person.area();
  if (person_area <= 0.0F) {
    return 0.0F;
  }
  const float overlap = (person & vehicle).area() / person_area;
  const float centre_x = person.x + (0.5F * person.width);
  const float feet_y = person.y + person.height;
  const bool centred = centre_x >= vehicle.x && centre_x <= vehicle.x + vehicle.width;
  const bool feet_low = feet_y >= vehicle.y + (options.min_feet_depth * vehicle.height);
  return centred && feet_low && overlap >= options.min_person_overlap ? overlap : 0.0F;
}

}  // namespace

std::vector<Detection> associate_riders(std::vector<Detection> detections,
                                        const RiderAssociationOptions& options) {
  std::vector<std::size_t> persons;
  std::vector<std::size_t> vehicles;
  for (std::size_t i = 0; i < detections.size(); ++i) {
    if (detections[i].twin_class == options.person_class) {
      persons.push_back(i);
    } else if (std::ranges::find(options.vehicle_classes, detections[i].twin_class) !=
               options.vehicle_classes.end()) {
      vehicles.push_back(i);
    }
  }
  if (persons.empty() || vehicles.empty()) {
    return detections;
  }

  Eigen::MatrixXd cost(static_cast<Eigen::Index>(persons.size()),
                       static_cast<Eigen::Index>(vehicles.size()));
  for (std::size_t p = 0; p < persons.size(); ++p) {
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
      const float overlap =
          rider_overlap(detections[persons[p]].box, detections[vehicles[v]].box, options);
      cost(static_cast<Eigen::Index>(p), static_cast<Eigen::Index>(v)) =
          overlap > 0.0F ? 1.0 - overlap : kForbiddenCost;
    }
  }

  std::vector<bool> absorbed(detections.size(), false);
  const auto assignment = core::solve_assignment(cost);
  for (std::size_t p = 0; p < assignment.size(); ++p) {
    const Eigen::Index v = assignment[p];
    if (v < 0 || cost(static_cast<Eigen::Index>(p), v) >= kForbiddenCost) {
      continue;
    }
    const Detection& person = detections[persons[p]];
    Detection& vehicle = detections[vehicles[static_cast<std::size_t>(v)]];
    vehicle.box = vehicle.box | person.box;
    vehicle.score = std::max(vehicle.score, person.score);
    vehicle.rider = true;
    absorbed[persons[p]] = true;
  }

  std::vector<Detection> result;
  result.reserve(detections.size());
  for (std::size_t i = 0; i < detections.size(); ++i) {
    if (!absorbed[i]) {
      result.push_back(std::move(detections[i]));
    }
  }
  return result;
}

}  // namespace jp::detection
