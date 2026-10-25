#include "jp/geometry/camera.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

namespace jp {
namespace {
constexpr double kDeg = M_PI / 180.0;

Eigen::Vector3d horizontalForward(const Eigen::Vector3d& n) {
  Eigen::Vector3d ez(0, 0, 1);
  Eigen::Vector3d f = ez - ez.dot(n) * n;
  return f.normalized();
}
}  // namespace

CameraModel::CameraModel(int width, int height, const Eigen::Matrix3d& K, const Eigen::Vector4d& plane_cam,
                         double bearing_deg, std::vector<double> dist)
    : width_(width), height_(height), K_(K), dist_(std::move(dist)), plane_(plane_cam), bearing_deg_(bearing_deg) {
  build();
}

CameraModel CameraModel::fromPose(int width, int height, double focal_px, double pitch, double roll, double height_m,
                                  double bearing_deg) {
  Eigen::Matrix3d K;
  K << focal_px, 0, (width - 1) / 2.0, 0, focal_px, (height - 1) / 2.0, 0, 0, 1;
  const double c = std::cos(pitch), s = std::sin(pitch);
  Eigen::Vector3d n(c * std::sin(roll), -c * std::cos(roll), -s);
  return CameraModel(width, height, K, Eigen::Vector4d(n.x(), n.y(), n.z(), height_m), bearing_deg);
}

void CameraModel::build() {
  Eigen::Vector3d n = plane_.head<3>();
  const double norm = n.norm();
  n /= norm;
  plane_.head<3>() = n;
  plane_[3] /= norm;
  Kinv_ = K_.inverse();
  const Eigen::Vector3d f = horizontalForward(n);
  const Eigen::Vector3d l = n.cross(f);
  Eigen::Matrix3d C;
  C.col(0) = f;
  C.col(1) = l;
  C.col(2) = n;
  const double b = bearing_deg_ * kDeg;
  Eigen::Matrix3d W;
  W.col(0) = Eigen::Vector3d(std::sin(b), std::cos(b), 0);
  W.col(1) = Eigen::Vector3d(-std::cos(b), std::sin(b), 0);
  W.col(2) = Eigen::Vector3d(0, 0, 1);
  R_gc_ = W * C.transpose();
}

double CameraModel::pitch() const { return std::asin(std::clamp(-plane_[2], -1.0, 1.0)); }
double CameraModel::roll() const { return std::atan2(plane_[0], -plane_[1]); }

Eigen::Vector3d CameraModel::ray(const Eigen::Vector2d& px) const { return Kinv_ * Eigen::Vector3d(px.x(), px.y(), 1); }

std::optional<Eigen::Vector3d> CameraModel::pixelToGround(const Eigen::Vector2d& px, double dz) const {
  const Eigen::Vector3d r = ray(px);
  const double denom = r.dot(plane_.head<3>());
  if (std::abs(denom) < 1e-12) return std::nullopt;
  const double t = (dz - plane_[3]) / denom;
  if (t <= 0 || !std::isfinite(t)) return std::nullopt;
  return cameraToGround(r * t);
}

Eigen::Vector3d CameraModel::cameraToGround(const Eigen::Vector3d& p) const {
  return R_gc_ * p + Eigen::Vector3d(0, 0, plane_[3]);
}

Eigen::Vector3d CameraModel::groundToCamera(const Eigen::Vector3d& p) const {
  return R_gc_.transpose() * (p - Eigen::Vector3d(0, 0, plane_[3]));
}

std::optional<Eigen::Vector2d> CameraModel::groundToPixel(const Eigen::Vector3d& p_ground) const {
  const Eigen::Vector3d pc = groundToCamera(p_ground);
  if (pc.z() < 0.1) return std::nullopt;
  const Eigen::Vector3d uv = K_ * pc;
  return Eigen::Vector2d(uv.x() / uv.z(), uv.y() / uv.z());
}

double CameraModel::horizonRow(double u) const {
  // Horizon: pixels whose ray is parallel to the ground, n . K^-1 [u v 1] = 0.
  const Eigen::RowVector3d a = plane_.head<3>().transpose() * Kinv_;
  return -(a[0] * u + a[2]) / a[1];
}

std::optional<Detection3D> CameraModel::lift(const ImageDetection& det, double pixel_sigma) const {
  auto bottom = pixelToGround(det.kp, det.dz);
  if (!bottom) return std::nullopt;
  const Eigen::Vector3d n = plane_.head<3>();
  Eigen::Vector3d r = ray(det.kp);
  r = (r - r.dot(n) * n).normalized();
  const Eigen::Vector3d s = n.cross(r);
  const Eigen::Vector3d h_cam = std::cos(det.alpha) * r + std::sin(det.alpha) * s;
  const Eigen::Vector3d h = R_gc_ * h_cam;

  Detection3D out;
  out.img = det;
  out.bottom = *bottom;
  out.yaw = std::atan2(h.y(), h.x());
  out.dims = det.dims;
  out.range = bottom->head<2>().norm();

  // Position covariance by finite differences of the ray-ground intersection.
  const double box_h = std::max(det.box2d[3] - det.box2d[1], 1.0);
  const double su = pixel_sigma + 0.01 * box_h, sv = pixel_sigma + 0.03 * box_h;
  Eigen::Matrix2d J;
  for (int k = 0; k < 2; ++k) {
    Eigen::Vector2d d = Eigen::Vector2d::Zero();
    d[k] = 1.0;
    auto p1 = pixelToGround(det.kp + d, det.dz);
    if (!p1) {
      J.col(k) = Eigen::Vector2d(1e3, 1e3);
      continue;
    }
    J.col(k) = (p1->head<2>() - bottom->head<2>());
  }
  Eigen::Matrix2d S = Eigen::Vector2d(su * su, sv * sv).asDiagonal();
  out.pos_cov = J * S * J.transpose() + Eigen::Matrix2d::Identity() * 0.01;
  return out;
}

std::optional<std::array<Eigen::Vector2d, 8>> CameraModel::projectBox(const Eigen::Vector3d& bottom, double yaw,
                                                                      const Eigen::Vector3d& dims) const {
  const Eigen::Vector3d x(std::cos(yaw), std::sin(yaw), 0), y(-std::sin(yaw), std::cos(yaw), 0), z(0, 0, 1);
  static constexpr int sgn[4][2] = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  std::array<Eigen::Vector2d, 8> out;
  for (int top = 0; top < 2; ++top) {
    for (int i = 0; i < 4; ++i) {
      const Eigen::Vector3d p = bottom + sgn[i][0] * dims[0] / 2 * x + sgn[i][1] * dims[1] / 2 * y + top * dims[2] * z;
      auto px = groundToPixel(p);
      if (!px) return std::nullopt;
      out[top * 4 + i] = *px;
    }
  }
  return out;
}

CameraModel CameraModel::rotated(const Eigen::Matrix3d& R_new_old) const {
  const Eigen::Vector3d n_new = R_new_old * plane_.head<3>();
  const Eigen::Vector3d f_new = horizontalForward(n_new);
  const Eigen::Vector3d f_world = R_gc_ * (R_new_old.transpose() * f_new);
  const double bearing = std::atan2(f_world.x(), f_world.y()) / kDeg;
  CameraModel out(width_, height_, K_, Eigen::Vector4d(n_new.x(), n_new.y(), n_new.z(), plane_[3]), bearing, dist_);
  out.anchor = anchor;
  return out;
}

nlohmann::json CameraModel::toJson() const {
  nlohmann::json j;
  j["width"] = width_;
  j["height"] = height_;
  j["K"] = {{K_(0, 0), K_(0, 1), K_(0, 2)}, {K_(1, 0), K_(1, 1), K_(1, 2)}, {K_(2, 0), K_(2, 1), K_(2, 2)}};
  j["dist"] = dist_;
  j["plane"] = {plane_[0], plane_[1], plane_[2], plane_[3]};
  j["bearing_deg"] = bearing_deg_;
  j["pitch_deg"] = pitch() / kDeg;
  j["roll_deg"] = roll() / kDeg;
  j["height_m"] = plane_[3];
  if (anchor.valid) j["anchor"] = {{"lat", anchor.lat}, {"lon", anchor.lon}, {"alt", anchor.alt}};
  return j;
}

CameraModel CameraModel::fromJson(const nlohmann::json& j) {
  Eigen::Matrix3d K;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) K(r, c) = j.at("K")[r][c].get<double>();
  const auto p = j.at("plane");
  CameraModel m(j.at("width"), j.at("height"), K, Eigen::Vector4d(p[0], p[1], p[2], p[3]),
                j.value("bearing_deg", 0.0), j.value("dist", std::vector<double>{}));
  if (j.contains("anchor")) {
    m.anchor = {j["anchor"].at("lat"), j["anchor"].at("lon"), j["anchor"].value("alt", 0.0), true};
  }
  return m;
}

namespace {
constexpr double kA = 6378137.0, kE2 = 6.69437999014e-3;
}

Eigen::Vector2d geoToEnu(const GeoAnchor& a, double lat, double lon) {
  const double phi = a.lat * kDeg;
  const double s = std::sin(phi);
  const double M = kA * (1 - kE2) / std::pow(1 - kE2 * s * s, 1.5);
  const double N = kA / std::sqrt(1 - kE2 * s * s);
  return {(lon - a.lon) * kDeg * N * std::cos(phi), (lat - a.lat) * kDeg * M};
}

std::pair<double, double> enuToGeo(const GeoAnchor& a, const Eigen::Vector2d& en) {
  const double phi = a.lat * kDeg;
  const double s = std::sin(phi);
  const double M = kA * (1 - kE2) / std::pow(1 - kE2 * s * s, 1.5);
  const double N = kA / std::sqrt(1 - kE2 * s * s);
  return {a.lat + en.y() / M / kDeg, a.lon + en.x() / (N * std::cos(phi)) / kDeg};
}

}  // namespace jp
