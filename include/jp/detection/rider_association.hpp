#pragma once

#include <string>
#include <vector>

#include "jp/detection/detection.hpp"

namespace jp::detection {

struct RiderAssociationOptions {
  std::string person_class = "pedestrian";
  std::vector<std::string> vehicle_classes = {"bicycle", "motorcycle"};
  /// Minimum share of the person box that must overlap the two-wheeler box.
  float min_person_overlap = 0.2F;
  /// The person's feet must be below this fraction of the two-wheeler box height (from its top).
  float min_feet_depth = 0.25F;
};

/// A detected person riding a detected bicycle or motorcycle becomes one object of the
/// two-wheeler's class: the box is the union of both boxes (so its bottom edge is the wheels'
/// ground contact), the score the higher of both, and `rider` is set. Pairs are assigned
/// one-to-one by overlap; unpaired persons and two-wheelers are returned unchanged.
[[nodiscard]] std::vector<Detection> associate_riders(std::vector<Detection> detections,
                                                      const RiderAssociationOptions& options = {});

}  // namespace jp::detection
