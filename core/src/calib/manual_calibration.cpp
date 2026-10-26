#include "jp/calib/manual_calibration.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>

namespace jp {
namespace {
constexpr double kDeg = M_PI / 180.0;

struct Pose {
  cv::Mat rvec, tvec;
  double rms = 1e18, max = 1e18;
};

// Best pose for a fixed focal length (planar target: IPPE, then iterative refinement).
Pose poseFor(double f, double cx, double cy, const std::vector<cv::Point3d>& obj, const std::vector<cv::Point2d>& img) {
  const cv::Matx33d K(f, 0, cx, 0, f, cy, 0, 0, 1);
  Pose p;
  try {
    if (!cv::solvePnP(obj, img, K, cv::noArray(), p.rvec, p.tvec, false, cv::SOLVEPNP_IPPE)) return p;
    cv::solvePnP(obj, img, K, cv::noArray(), p.rvec, p.tvec, true, cv::SOLVEPNP_ITERATIVE);
  } catch (const cv::Exception&) {
    return p;
  }
  std::vector<cv::Point2d> proj;
  cv::projectPoints(obj, p.rvec, p.tvec, K, cv::noArray(), proj);
  double s = 0, mx = 0;
  for (size_t i = 0; i < img.size(); ++i) {
    const double e = cv::norm(proj[i] - img[i]);
    s += e * e;
    mx = std::max(mx, e);
  }
  // The camera must be above the ground and every point in front of it.
  cv::Matx33d R;
  cv::Rodrigues(p.rvec, R);
  const cv::Vec3d t(p.tvec);
  const cv::Vec3d C = -(R.t() * t);
  bool front = C[2] > 0.3;
  for (const auto& o : obj) front = front && (R * cv::Vec3d(o.x, o.y, o.z) + t)[2] > 0.5;
  if (!front) return p;
  p.rms = std::sqrt(s / img.size());
  p.max = mx;
  return p;
}

// Focal length from the ground homography (principal point known): r1 . r2 = 0 and |r1| = |r2|.
std::optional<double> focalFromHomography(const std::vector<cv::Point3d>& obj, const std::vector<cv::Point2d>& img,
                                          double cx, double cy) {
  std::vector<cv::Point2d> src, dst;
  for (size_t i = 0; i < obj.size(); ++i) {
    src.emplace_back(obj[i].x, obj[i].y);
    dst.emplace_back(img[i].x - cx, img[i].y - cy);
  }
  const cv::Mat H = cv::findHomography(src, dst, 0);
  if (H.empty()) return std::nullopt;
  const cv::Matx33d h(H);
  const double a1 = h(0, 0), a2 = h(1, 0), a3 = h(2, 0), b1 = h(0, 1), b2 = h(1, 1), b3 = h(2, 1);
  std::vector<double> f2;
  if (std::abs(a3 * b3) > 1e-12) f2.push_back(-(a1 * b1 + a2 * b2) / (a3 * b3));
  if (std::abs(a3 * a3 - b3 * b3) > 1e-12) f2.push_back(-(a1 * a1 + a2 * a2 - b1 * b1 - b2 * b2) / (a3 * a3 - b3 * b3));
  double sum = 0;
  int n = 0;
  for (double v : f2)
    if (v > 0) {
      sum += std::sqrt(v);
      ++n;
    }
  if (!n) return std::nullopt;
  return sum / n;
}
}  // namespace

ManualCalibrationResult calibrateFromCorrespondences(int width, int height, const std::vector<GroundCorrespondence>& pts,
                                                     const GeoAnchor& anchor, std::optional<double> focal_px,
                                                     const std::vector<double>& dist) {
  ManualCalibrationResult res;
  if (pts.size() < 4) {
    res.message = "at least 4 point pairs are needed";
    return res;
  }
  if (!anchor.valid) {
    res.message = "camera position (lat/lon) is not configured";
    return res;
  }
  const double cx = (width - 1) / 2.0, cy = (height - 1) / 2.0;
  std::vector<cv::Point3d> obj;
  std::vector<cv::Point2d> img;
  for (const auto& p : pts) {
    const Eigen::Vector2d en = geoToEnu(anchor, p.lat, p.lon);
    obj.emplace_back(en.x(), en.y(), 0.0);
    img.emplace_back(p.px.x(), p.px.y());
  }
  if (focal_px && !dist.empty()) {  // clicked on the raw frame: undistort with the known intrinsics
    const cv::Matx33d K(*focal_px, 0, cx, 0, *focal_px, cy, 0, 0, 1);
    std::vector<cv::Point2d> und;
    cv::undistortPoints(img, und, K, dist, cv::noArray(), K);
    img = und;
  }
  Pose best;
  double best_f = 0;
  auto tryF = [&](double f) {
    Pose p = poseFor(f, cx, cy, obj, img);
    if (p.rms < best.rms) {
      best = p;
      best_f = f;
    }
    return p.rms;
  };
  if (focal_px) {
    tryF(*focal_px);
  } else {
    // Log grid (seeded with the homography estimate), then golden-section refinement.
    if (auto f0 = focalFromHomography(obj, img, cx, cy); f0 && *f0 > 0.2 * width && *f0 < 10.0 * width) tryF(*f0);
    const double l0 = std::log(0.3 * width), l1 = std::log(8.0 * width);
    for (int i = 0; i <= 80; ++i) tryF(std::exp(l0 + (l1 - l0) * i / 80));
    if (best_f > 0) {
      double a = std::log(best_f) - (l1 - l0) / 80, b = std::log(best_f) + (l1 - l0) / 80;
      const double g = (std::sqrt(5.0) - 1) / 2;
      double c = b - g * (b - a), d = a + g * (b - a);
      double fc = tryF(std::exp(c)), fd = tryF(std::exp(d));
      for (int it = 0; it < 40; ++it) {
        if (fc < fd) {
          b = d;
          d = c;
          fd = fc;
          c = b - g * (b - a);
          fc = tryF(std::exp(c));
        } else {
          a = c;
          c = d;
          fc = fd;
          d = a + g * (b - a);
          fd = tryF(std::exp(d));
        }
      }
    }
  }
  if (best_f <= 0 || best.rms > 1e17) {
    res.message = "no camera pose explains these points (check that every map point is on the ground)";
    return res;
  }
  cv::Matx33d Rcv;
  cv::Rodrigues(best.rvec, Rcv);
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) R(i, j) = Rcv(i, j);
  const Eigen::Vector3d t(best.tvec.at<double>(0), best.tvec.at<double>(1), best.tvec.at<double>(2));
  const Eigen::Vector3d C = -R.transpose() * t;  // camera centre in the anchor's ENU frame
  const Eigen::Vector3d n = R * Eigen::Vector3d(0, 0, 1);  // ground up in camera coordinates
  const Eigen::Vector3d fwd = R.transpose() * Eigen::Vector3d(0, 0, 1);
  const double bearing = std::fmod(std::atan2(fwd.x(), fwd.y()) / kDeg + 360.0, 360.0);
  Eigen::Matrix3d K;
  K << best_f, 0, cx, 0, best_f, cy, 0, 0, 1;
  res.camera = CameraModel(width, height, K, Eigen::Vector4d(n.x(), n.y(), n.z(), C.z()), bearing, dist);
  const auto [lat, lon] = enuToGeo(anchor, C.head<2>());
  res.camera.anchor = {lat, lon, anchor.alt, true};
  res.rms_px = best.rms;
  res.max_px = best.max;
  const double tol = std::max(4.0, 0.004 * width);
  res.ok = best.rms <= tol && C.z() > 0.5 && C.z() < 150.0;
  char buf[256];
  std::snprintf(buf, sizeof(buf), "f=%.0f px, pitch=%.2f deg, roll=%.2f deg, height=%.2f m, bearing=%.1f deg, "
                "camera %.1f m from the configured position, reprojection rms %.1f px (max %.1f)",
                best_f, res.camera.pitch() / kDeg, res.camera.roll() / kDeg, C.z(), bearing, C.head<2>().norm(),
                best.rms, best.max);
  res.message = buf;
  if (!res.ok && best.rms > tol)
    res.message = std::string("points are inconsistent (") + buf + "); check the pairs with the largest error";
  spdlog::info("Manual calibration from {} points: {}", pts.size(), res.message);
  return res;
}

}  // namespace jp
