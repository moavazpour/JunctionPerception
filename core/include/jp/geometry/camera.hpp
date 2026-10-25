#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <vector>

#include "jp/common/types.hpp"

namespace jp {

// WGS84 anchor of the local ground frame (the ground point below the camera).
struct GeoAnchor {
  double lat = 0, lon = 0, alt = 0;
  bool valid = false;
};

// Pinhole camera above a locally planar ground.
//
// Frames:
//   camera  - OpenCV (x right, y down, z forward), undistorted pixels.
//   ground  - local ENU: x east, y north, z up, origin on the ground directly below the camera.
// The ground plane in camera coordinates is n.X + d = 0 with n pointing up, so d is the camera height.
class CameraModel {
 public:
  CameraModel() = default;

  // Build from intrinsics, ground plane (in camera frame) and the compass bearing (deg, clockwise from
  // north) of the camera's horizontal viewing direction.
  CameraModel(int width, int height, const Eigen::Matrix3d& K, const Eigen::Vector4d& plane_cam,
              double bearing_deg, std::vector<double> dist = {});

  // Convenience: pitch (down positive, rad), roll (rad), height (m), focal (px), principal point at centre.
  static CameraModel fromPose(int width, int height, double focal_px, double pitch, double roll, double height_m,
                              double bearing_deg);

  int width() const { return width_; }
  int height() const { return height_; }
  const Eigen::Matrix3d& K() const { return K_; }
  const std::vector<double>& distortion() const { return dist_; }
  const Eigen::Vector4d& planeCam() const { return plane_; }
  double cameraHeight() const { return plane_[3]; }
  double bearingDeg() const { return bearing_deg_; }
  const Eigen::Matrix3d& R_ground_cam() const { return R_gc_; }
  Eigen::Vector3d cameraPositionGround() const { return {0, 0, plane_[3]}; }
  double pitch() const;  // down positive
  double roll() const;

  GeoAnchor anchor;

  Eigen::Vector3d ray(const Eigen::Vector2d& px) const;
  // Ground-frame point where the pixel ray meets the plane raised by dz. Empty if above the horizon.
  std::optional<Eigen::Vector3d> pixelToGround(const Eigen::Vector2d& px, double dz = 0.0) const;
  std::optional<Eigen::Vector2d> groundToPixel(const Eigen::Vector3d& p_ground) const;
  Eigen::Vector3d groundToCamera(const Eigen::Vector3d& p) const;
  Eigen::Vector3d cameraToGround(const Eigen::Vector3d& p) const;
  // Image row of the horizon at a given column.
  double horizonRow(double u) const;

  // Lift a calibration-free detection; pixel_sigma drives the reported position covariance.
  std::optional<Detection3D> lift(const ImageDetection& det, double pixel_sigma = 1.5) const;

  // 8 corners of a ground-frame box (bottom 4 then top 4) projected to pixels (invalid if behind).
  std::optional<std::array<Eigen::Vector2d, 8>> projectBox(const Eigen::Vector3d& bottom, double yaw,
                                                           const Eigen::Vector3d& dims) const;

  // Small rotation correction (camera moved): R_new_old applied to the camera frame.
  CameraModel rotated(const Eigen::Matrix3d& R_new_old) const;

  nlohmann::json toJson() const;
  static CameraModel fromJson(const nlohmann::json& j);
  bool valid() const { return width_ > 0; }

 private:
  void build();
  int width_ = 0, height_ = 0;
  Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity(), Kinv_ = Eigen::Matrix3d::Identity();
  std::vector<double> dist_;
  Eigen::Vector4d plane_ = Eigen::Vector4d(0, -1, 0, 1);
  double bearing_deg_ = 0;
  Eigen::Matrix3d R_gc_ = Eigen::Matrix3d::Identity();  // camera -> ground rotation
};

// Local tangent-plane conversion around an anchor (accurate to centimetres within a few km).
Eigen::Vector2d geoToEnu(const GeoAnchor& a, double lat, double lon);
std::pair<double, double> enuToGeo(const GeoAnchor& a, const Eigen::Vector2d& en);

}  // namespace jp
