#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace jp {

using Clock = double;  // seconds since epoch (stream time)

enum class ObjectClass : std::uint8_t {
  Car = 0,
  Bus,
  Truck,
  Trailer,
  Pedestrian,
  Bicycle,
  EScooter,
  Motorcycle,
  Count
};

constexpr std::array<std::string_view, static_cast<size_t>(ObjectClass::Count)> kClassNames = {
    "car", "bus", "truck", "trailer", "pedestrian", "bicycle", "escooter", "motorcycle"};

inline std::string_view className(ObjectClass c) { return kClassNames[static_cast<size_t>(c)]; }
// Prior dimensions (l, w, h) in metres; must match training/jp3d/geometry.py.
inline Eigen::Vector3d classPriorDims(ObjectClass c) {
  static const Eigen::Vector3d d[] = {{4.45, 1.90, 1.60}, {12.0, 2.60, 3.20}, {6.80, 2.50, 3.20}, {10.5, 2.60, 3.40},
                                      {0.65, 0.65, 1.72}, {1.75, 0.65, 1.70}, {1.15, 0.60, 1.80}, {2.10, 0.85, 1.60}};
  return d[static_cast<size_t>(c)];
}
inline bool isVulnerable(ObjectClass c) {
  return c == ObjectClass::Pedestrian || c == ObjectClass::Bicycle || c == ObjectClass::EScooter ||
         c == ObjectClass::Motorcycle;
}
inline bool isVehicle(ObjectClass c) {
  return c == ObjectClass::Car || c == ObjectClass::Bus || c == ObjectClass::Truck || c == ObjectClass::Trailer ||
         c == ObjectClass::Motorcycle;
}

// Calibration-free network output (image space of the original, undistorted frame).
struct ImageDetection {
  float score = 0;
  ObjectClass cls = ObjectClass::Car;
  Eigen::Vector2d kp;        // projected bottom centre of the 3D box (pixels)
  Eigen::Vector4d box2d;     // x1, y1, x2, y2 (pixels)
  double dz = 0;             // height of bottom centre above ground (m)
  Eigen::Vector3d dims;      // l, w, h (m)
  double alpha = 0;          // ground-relative observation angle (rad)
  // Keypoint models: projected bottom centre, footprint corners FL FR RR RL, top corners FL FR RR RL
  // as (u, v, confidence). When present, lifting fits the footprint directly instead of using priors.
  bool has_kpts = false;
  std::array<Eigen::Vector3d, 9> kpts{};
};

// Detection lifted to the local ground frame (ENU-aligned, metres, z up, origin under the camera).
struct Detection3D {
  ImageDetection img;
  Eigen::Vector3d bottom;    // footprint centre on the ground
  double yaw = 0;            // heading in ground frame (rad, counter-clockwise from +x/east)
  Eigen::Vector3d dims;
  double range = 0;          // horizontal distance from the camera (m)
  Eigen::Matrix2d pos_cov = Eigen::Matrix2d::Identity();  // position uncertainty from pixel noise
};

struct Frame {
  std::uint64_t index = 0;
  Clock stamp = 0;
  int width = 0, height = 0;
};

}  // namespace jp
