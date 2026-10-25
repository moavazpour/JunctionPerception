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
