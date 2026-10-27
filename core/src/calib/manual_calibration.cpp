#include "jp/calib/manual_calibration.hpp"

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <opencv2/calib3d.hpp>

namespace jp {
namespace {
constexpr double kDeg = M_PI / 180.0;
constexpr int kN = 7;  // rvec (3), camera centre (3), log focal
using Vec7 = Eigen::Matrix<double, kN, 1>;

// Field of view along the long image side for each lens hint: centre (deg) and log-focal sigma.
struct LensPrior {
  double fov_deg, log_sigma;
};
LensPrior lensPrior(const std::string& lens) {
  if (lens == "phone_wide") return {66.0, 0.12};
  if (lens == "phone_2x") return {37.0, 0.15};
  if (lens == "phone_3x") return {25.0, 0.18};
  if (lens == "cctv") return {55.0, 0.45};
  return {55.0, 0.70};  // unknown: only rules out absurd focal lengths
}

double focalFromFov(double long_side, double fov_deg) { return long_side / 2.0 / std::tan(fov_deg * kDeg / 2.0); }
double fovFromFocal(double long_side, double f) { return 2.0 * std::atan(long_side / 2.0 / f) / kDeg; }

struct Cam {
  Eigen::Matrix3d R;  // world (ENU) -> camera
  Eigen::Vector3d C;  // camera centre in ENU
  double f, cx, cy;

  static Cam from(const Vec7& x, double cx, double cy) {
    Cam c;
    const Eigen::Vector3d r = x.head<3>();
    const double a = r.norm();
    c.R = a < 1e-12 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(a, r / a).toRotationMatrix();
    c.C = x.segment<3>(3);
    c.f = std::exp(x[6]);
    c.cx = cx;
    c.cy = cy;
    return c;
  }
  std::optional<Eigen::Vector2d> project(const Eigen::Vector3d& X) const {
    const Eigen::Vector3d p = R * (X - C);
    if (p.z() < 0.1) return std::nullopt;
    return Eigen::Vector2d(f * p.x() / p.z() + cx, f * p.y() / p.z() + cy);
  }
  // Ground point (z = 0) seen at a pixel; empty above the horizon.
  std::optional<Eigen::Vector2d> backproject(const Eigen::Vector2d& px) const {
    const Eigen::Vector3d d = R.transpose() * Eigen::Vector3d((px.x() - cx) / f, (px.y() - cy) / f, 1.0);
    if (d.z() > -1e-6) return std::nullopt;
    const double t = -C.z() / d.z();
    if (t > 5000) return std::nullopt;
    return (C + t * d).head<2>();
  }
  // Pixels per metre on the ground around a point (for weighting map-reading errors).
  double pxPerMetre(const Eigen::Vector2d& g) const {
    const auto a = project({g.x() - 0.5, g.y(), 0}), b = project({g.x() + 0.5, g.y(), 0});
    const auto c = project({g.x(), g.y() - 0.5, 0}), d = project({g.x(), g.y() + 0.5, 0});
    if (!a || !b || !c || !d) return 1.0;
    return 0.5 * ((*b - *a).norm() + (*d - *c).norm());
  }
};

struct Problem {
  const ManualCalibrationInput& in;
  double cx, cy, long_side;
  std::vector<Eigen::Vector2d> img;    // point pixels (undistorted)
  std::vector<Eigen::Vector2d> world;  // point ENU
  std::vector<std::vector<Eigen::Vector2d>> line_px;
  std::vector<std::array<Eigen::Vector2d, 2>> line_world;
  std::vector<VerticalLine> verts;
  double click = 1.5;
  double logf0 = 0, logf_sigma = 1;
  double h0 = 8, h_sigma = 8;
  // Per-measurement sigma (px) and robust weight, refreshed between rounds.
  std::vector<double> pt_sigma, pt_w, ln_sigma, ln_w;

  // Normalised residuals: points (2 each), line pixels (1 each), verticals (1 each), then the priors.
  Eigen::VectorXd residuals(const Vec7& x) const {
    const Cam c = Cam::from(x, cx, cy);
    std::vector<double> r;
    r.reserve(2 * img.size() + 64);
    constexpr double kBad = 1e3;
    for (size_t i = 0; i < img.size(); ++i) {
      const auto p = c.project({world[i].x(), world[i].y(), 0});
      const double s = std::sqrt(pt_w[i]) / pt_sigma[i];
      if (!p) {
        r.push_back(kBad);
        r.push_back(kBad);
        continue;
      }
      r.push_back((p->x() - img[i].x()) * s);
      r.push_back((p->y() - img[i].y()) * s);
    }
    for (size_t j = 0; j < line_px.size(); ++j) {
      const auto a = c.project({line_world[j][0].x(), line_world[j][0].y(), 0});
      const auto b = c.project({line_world[j][1].x(), line_world[j][1].y(), 0});
      // Each line counts like about two points, however many pixels were traced.
      const double s = std::sqrt(ln_w[j] * 2.0 / std::max<size_t>(line_px[j].size(), 2)) / ln_sigma[j];
      if (!a || !b || (*b - *a).norm() < 1e-6) {
        for (size_t k = 0; k < line_px[j].size(); ++k) r.push_back(kBad);
        continue;
      }
      const Eigen::Vector2d d = (*b - *a).normalized(), n(-d.y(), d.x());
      for (const auto& q : line_px[j]) r.push_back((q - *a).dot(n) * s);
    }
    if (!verts.empty()) {
      const Eigen::Vector3d v = c.R * Eigen::Vector3d::UnitZ();  // vertical direction in the camera
      for (const auto& seg : verts) {
        const Eigen::Vector2d m = 0.5 * (seg[0] + seg[1]), d = seg[1] - seg[0];
        const double L = d.norm();
        // Image direction of verticals at m: towards the vertical vanishing point.
        Eigen::Vector2d dir;
        if (std::abs(v.z()) < 1e-9) {
          dir = v.head<2>();
        } else {
          const Eigen::Vector2d vp(c.f * v.x() / v.z() + cx, c.f * v.y() / v.z() + cy);
          dir = vp - m;
          if (v.z() < 0) dir = -dir;
        }
        if (dir.norm() < 1e-9 || L < 1e-6) {
          r.push_back(0);
          continue;
        }
        const double sin_a = (d.x() * dir.y() - d.y() * dir.x()) / (L * dir.norm());
        r.push_back(0.5 * L * sin_a / (2.0 * click));
      }
    }
    r.push_back((x[6] - logf0) / logf_sigma);
    r.push_back((x[5] - h0) / h_sigma);
    r.push_back(x[3] / in.anchor_sigma_m);
    r.push_back(x[4] / in.anchor_sigma_m);
    // The camera is above the ground.
    r.push_back(x[5] < 1.0 ? (1.0 - x[5]) * 10.0 : 0.0);
    return Eigen::Map<Eigen::VectorXd>(r.data(), static_cast<Eigen::Index>(r.size()));
  }

  Eigen::MatrixXd jacobian(const Vec7& x, const Eigen::VectorXd& r0) const {
    Eigen::MatrixXd J(r0.size(), kN);
    for (int k = 0; k < kN; ++k) {
      Vec7 xp = x;
      const double h = 1e-6 * std::max(1.0, std::abs(x[k]));
      xp[k] += h;
      J.col(k) = (residuals(xp) - r0) / h;
    }
    return J;
  }

  double lm(Vec7& x, int iters) const {
    Eigen::VectorXd r = residuals(x);
    double cost = r.squaredNorm(), lambda = 1e-3;
    for (int it = 0; it < iters; ++it) {
      const Eigen::MatrixXd J = jacobian(x, r);
      const Eigen::Matrix<double, kN, kN> H = J.transpose() * J;
      const Vec7 g = J.transpose() * r;
      bool improved = false;
      for (int tries = 0; tries < 10; ++tries) {
        Eigen::Matrix<double, kN, kN> A = H;
        A.diagonal() += lambda * (H.diagonal().array() + 1e-9).matrix();
        const Vec7 dx = A.ldlt().solve(-g);
        if (!dx.allFinite()) {
          lambda *= 10;
          continue;
        }
        const Vec7 xn = x + dx;
        const Eigen::VectorXd rn = residuals(xn);
        const double cn = rn.squaredNorm();
        if (cn < cost) {
          const double rel = (cost - cn) / std::max(cost, 1e-12);
          x = xn;
          r = rn;
          cost = cn;
          lambda = std::max(lambda / 3, 1e-9);
          improved = true;
          if (rel < 1e-10) return cost;
          break;
        }
        lambda *= 10;
      }
      if (!improved) break;
    }
    return cost;
  }

  // Sigma per measurement: click accuracy plus the map error seen at that distance; Huber weights.
  void reweight(const Vec7& x, bool robust) {
    const Cam c = Cam::from(x, cx, cy);
    const double m = in.map_sigma_m;
    for (size_t i = 0; i < img.size(); ++i) {
      const double s = c.pxPerMetre(world[i]);
      pt_sigma[i] = std::sqrt(click * click + s * m * s * m);
      pt_w[i] = 1.0;
      if (robust) {
        const auto p = c.project({world[i].x(), world[i].y(), 0});
        const double e = p ? (*p - img[i]).norm() / pt_sigma[i] : 1e3;
        constexpr double k = 2.5;
        pt_w[i] = e <= k ? 1.0 : k / e;
      }
    }
    for (size_t j = 0; j < line_px.size(); ++j) {
      const double s = c.pxPerMetre(0.5 * (line_world[j][0] + line_world[j][1]));
      ln_sigma[j] = std::sqrt(click * click + s * m * s * m);
      ln_w[j] = 1.0;
    }
  }
};

Cam toCam(const cv::Mat& rvec, const cv::Mat& tvec, double f, double cx, double cy) {
  cv::Matx33d Rcv;
  cv::Rodrigues(rvec, Rcv);
  Cam c;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) c.R(i, j) = Rcv(i, j);
  const Eigen::Vector3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  c.C = -c.R.transpose() * t;
  c.f = f;
  c.cx = cx;
  c.cy = cy;
  return c;
}

Vec7 toParams(const Cam& c) {
  Vec7 x;
  const Eigen::AngleAxisd aa(c.R);
  x.head<3>() = aa.angle() * aa.axis();
  x.segment<3>(3) = c.C;
  x[6] = std::log(c.f);
  return x;
}

// Focal length from the ground homography (principal point known): r1 . r2 = 0 and |r1| = |r2|.
std::optional<double> focalFromHomography(const std::vector<Eigen::Vector2d>& world, const std::vector<Eigen::Vector2d>& img,
                                          double cx, double cy) {
  std::vector<cv::Point2d> src, dst;
  for (size_t i = 0; i < world.size(); ++i) {
    src.emplace_back(world[i].x(), world[i].y());
    dst.emplace_back(img[i].x() - cx, img[i].y() - cy);
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

double distToLine(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  const Eigen::Vector2d d = b - a;
  const double L = d.norm();
  if (L < 1e-9) return (p - a).norm();
  return std::abs(d.x() * (p.y() - a.y()) - d.y() * (p.x() - a.x())) / L;
}
}  // namespace

ManualCalibrationResult calibrateManual(const ManualCalibrationInput& in) {
  ManualCalibrationResult res;
  if (in.points.size() < 4) {
    res.message = "at least 4 point pairs are needed";
    return res;
  }
  if (!in.anchor.valid) {
    res.message = "camera position (lat/lon) is not configured";
    return res;
  }
  Problem P{in, (in.width - 1) / 2.0, (in.height - 1) / 2.0, static_cast<double>(std::max(in.width, in.height))};
  P.click = in.click_sigma_px > 0 ? in.click_sigma_px : std::max(1.5, 1.0 * in.width / 960.0);

  // Pixels clicked on the raw frame are undistorted with known intrinsics.
  auto undistort = [&](std::vector<Eigen::Vector2d> px) {
    if (!in.focal_px || in.dist.empty() || px.empty()) return px;
    const cv::Matx33d K(*in.focal_px, 0, P.cx, 0, *in.focal_px, P.cy, 0, 0, 1);
    std::vector<cv::Point2d> a, b;
    for (const auto& p : px) a.emplace_back(p.x(), p.y());
    cv::undistortPoints(a, b, K, in.dist, cv::noArray(), K);
    for (size_t i = 0; i < px.size(); ++i) px[i] = {b[i].x, b[i].y};
    return px;
  };
  {
    std::vector<Eigen::Vector2d> px;
    for (const auto& p : in.points) {
      px.push_back(p.px);
      P.world.push_back(geoToEnu(in.anchor, p.lat, p.lon));
    }
    P.img = undistort(px);
  }
  for (const auto& l : in.lines) {
    if (l.px.size() < 2) continue;
    P.line_px.push_back(undistort(l.px));
    P.line_world.push_back({geoToEnu(in.anchor, l.geo[0].x(), l.geo[0].y()), geoToEnu(in.anchor, l.geo[1].x(), l.geo[1].y())});
  }
  for (const auto& v : in.verticals) {
    auto u = undistort({v[0], v[1]});
    if ((u[1] - u[0]).norm() > 10) P.verts.push_back({u[0], u[1]});
  }
  P.pt_sigma.assign(P.img.size(), 1);
  P.pt_w.assign(P.img.size(), 1);
  P.ln_sigma.assign(P.line_px.size(), 1);
  P.ln_w.assign(P.line_px.size(), 1);

  const LensPrior lens = lensPrior(in.lens);
  if (in.focal_px) {
    P.logf0 = std::log(*in.focal_px);
    P.logf_sigma = 1e-4;
  } else {
    P.logf0 = std::log(focalFromFov(P.long_side, lens.fov_deg));
    P.logf_sigma = lens.log_sigma;
  }
  if (in.height_m) {
    P.h0 = *in.height_m;
    P.h_sigma = std::max(in.height_sigma, 0.2);
  }

  // Starts: a log grid of focal lengths (plus the homography estimate), pose from IPPE at each.
  std::vector<double> focals;
  if (in.focal_px) {
    focals.push_back(*in.focal_px);
  } else {
    for (int i = 0; i <= 27; ++i) focals.push_back(focalFromFov(P.long_side, 10.0 + i * 105.0 / 27));
    if (auto f0 = focalFromHomography(P.world, P.img, P.cx, P.cy); f0 && *f0 > 0.2 * P.long_side && *f0 < 12 * P.long_side)
      focals.push_back(*f0);
  }
  std::vector<cv::Point3d> obj;
  std::vector<cv::Point2d> im;
  for (size_t i = 0; i < P.img.size(); ++i) {
    obj.emplace_back(P.world[i].x(), P.world[i].y(), 0);
    im.emplace_back(P.img[i].x(), P.img[i].y());
  }
  std::vector<std::pair<double, Vec7>> starts;
  auto tryStart = [&](const Cam& c) {
    if (c.C.z() < 0.3) return;
    Vec7 x = toParams(c);
    P.reweight(x, false);
    const double cost = P.lm(x, 25);
    if (std::isfinite(cost)) starts.emplace_back(cost, x);
  };
  // Look-at start (robust to bad pairs): camera above the configured position at the height estimate,
  // looking at the centroid of the points.
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& w : P.world) centroid += w;
  centroid /= P.world.size();
  auto lookAt = [&](double f) {
    Cam c;
    c.C = Eigen::Vector3d(0, 0, P.h0);
    const Eigen::Vector3d fwd = (Eigen::Vector3d(centroid.x(), centroid.y(), 0) - c.C).normalized();
    const Eigen::Vector3d right = fwd.cross(Eigen::Vector3d::UnitZ()).normalized();
    const Eigen::Vector3d down = fwd.cross(right);
    c.R.row(0) = right.transpose();
    c.R.row(1) = down.transpose();
    c.R.row(2) = fwd.transpose();
    c.f = f;
    c.cx = P.cx;
    c.cy = P.cy;
    return c;
  };
  for (size_t i = 0; i < focals.size(); ++i) {
    const double f = focals[i];
    const cv::Matx33d K(f, 0, P.cx, 0, f, P.cy, 0, 0, 1);
    std::vector<cv::Mat> rvecs, tvecs;
    try {
      // Both IPPE solutions: with a bad pair the lower-error one can be the mirror pose below the ground.
      cv::solvePnPGeneric(obj, im, K, cv::noArray(), rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    } catch (const cv::Exception&) {
    }
    for (size_t k = 0; k < rvecs.size(); ++k) tryStart(toCam(rvecs[k], tvecs[k], f, P.cx, P.cy));
    if (i % 3 == 0) tryStart(lookAt(f));
  }
  if (starts.empty()) {
    res.message = "no camera pose explains these points (check that every map point is on the ground)";
    return res;
  }
  std::sort(starts.begin(), starts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  double best_cost = 1e300;
  Vec7 best = starts.front().second;
  for (size_t k = 0; k < std::min<size_t>(4, starts.size()); ++k) {
    Vec7 x = starts[k].second;
    double cost = 0;
    for (int round = 0; round < 3; ++round) {
      P.reweight(x, round > 0);
      cost = P.lm(x, 100);
    }
    if (cost < best_cost) {
      best_cost = cost;
      best = x;
    }
  }
  P.reweight(best, true);
  const Cam cam = Cam::from(best, P.cx, P.cy);

  // Residuals, measured both in the image and on the ground.
  std::vector<double> ground_err;
  double sum_px = 0;
  bool all_front = true;
  for (size_t i = 0; i < P.img.size(); ++i) {
    const auto p = cam.project({P.world[i].x(), P.world[i].y(), 0});
    const auto g = cam.backproject(P.img[i]);
    all_front = all_front && p.has_value();
    const double epx = p ? (*p - P.img[i]).norm() : 1e3;
    const double em = g ? (*g - P.world[i]).norm() : 999.0;
    sum_px += epx * epx;
    res.max_px = std::max(res.max_px, epx);
    ground_err.push_back(em);
    res.residuals.push_back({"point", static_cast<int>(i), epx, em});
  }
  res.rms_px = std::sqrt(sum_px / P.img.size());
  for (size_t j = 0; j < P.line_px.size(); ++j) {
    const auto a = cam.project({P.line_world[j][0].x(), P.line_world[j][0].y(), 0});
    const auto b = cam.project({P.line_world[j][1].x(), P.line_world[j][1].y(), 0});
    double spx = 0, sm = 0;
    for (const auto& q : P.line_px[j]) {
      const double e = a && b ? distToLine(q, *a, *b) : 1e3;
      const auto g = cam.backproject(q);
      const double m = g ? distToLine(*g, P.line_world[j][0], P.line_world[j][1]) : 999.0;
      spx += e * e;
      sm += m * m;
    }
    const double n = static_cast<double>(P.line_px[j].size());
    res.residuals.push_back({"line", static_cast<int>(j), std::sqrt(spx / n), std::sqrt(sm / n)});
  }
  for (size_t j = 0; j < P.verts.size(); ++j) {
    const Eigen::Vector3d v = cam.R * Eigen::Vector3d::UnitZ();
    const auto& seg = P.verts[j];
    const Eigen::Vector2d m = 0.5 * (seg[0] + seg[1]), d = seg[1] - seg[0];
    double e = 0;
    if (std::abs(v.z()) > 1e-9) {
      Eigen::Vector2d dir = Eigen::Vector2d(cam.f * v.x() / v.z() + P.cx, cam.f * v.y() / v.z() + P.cy) - m;
      e = 0.5 * d.norm() * std::abs(d.x() * dir.y() - d.y() * dir.x()) / (d.norm() * dir.norm() + 1e-12);
    }
    res.residuals.push_back({"vertical", static_cast<int>(j), e, 0});
  }
  {
    std::vector<double> sq;
    for (double e : ground_err) sq.push_back(e * e);
    res.ground_rms_m = std::sqrt(std::accumulate(sq.begin(), sq.end(), 0.0) / sq.size());
    res.ground_max_m = *std::max_element(ground_err.begin(), ground_err.end());
  }

  // Camera model in the frame of the ground point below the solved camera.
  const Eigen::Vector3d n = cam.R * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d fwd = cam.R.transpose() * Eigen::Vector3d::UnitZ();
  const double bearing = std::fmod(std::atan2(fwd.x(), fwd.y()) / kDeg + 360.0, 360.0);
  Eigen::Matrix3d K;
  K << cam.f, 0, P.cx, 0, cam.f, P.cy, 0, 0, 1;
  res.camera = CameraModel(in.width, in.height, K, Eigen::Vector4d(n.x(), n.y(), n.z(), cam.C.z()), bearing, in.dist);
  const auto [lat, lon] = enuToGeo(in.anchor, cam.C.head<2>());
  res.camera.anchor = {lat, lon, in.anchor.alt, true};
  res.focal_px = cam.f;
  res.fov_deg = fovFromFocal(P.long_side, cam.f);

  // Expected position error from the calibration (parameter covariance, inflated by the misfit).
  {
    const Eigen::VectorXd r = P.residuals(best);
    const Eigen::MatrixXd J = P.jacobian(best, r);
    Eigen::Matrix<double, kN, kN> H = J.transpose() * J;
    H.diagonal().array() += 1e-9;
    const Eigen::Matrix<double, kN, kN> S = H.inverse();
    const double dof = std::max<double>(1.0, static_cast<double>(r.size()) - kN);
    const double inflate = std::max(1.0, std::sqrt(r.squaredNorm() / dof));
    const Eigen::Vector2d fh = Eigen::Vector2d(fwd.x(), fwd.y()).normalized();
    for (double rng : {10.0, 20.0, 35.0, 50.0, 75.0, 100.0, 150.0, 200.0}) {
      const Eigen::Vector2d G = cam.C.head<2>() + rng * fh;
      const auto px = cam.project({G.x(), G.y(), 0});
      if (!px || px->x() < 0 || px->y() < 0 || px->x() >= in.width || px->y() >= in.height) continue;
      Eigen::Matrix<double, 2, kN> JG;
      bool okj = true;
      for (int k = 0; k < kN && okj; ++k) {
        Vec7 xp = best;
        const double h = 1e-6 * std::max(1.0, std::abs(best[k]));
        xp[k] += h;
        const auto g1 = Cam::from(xp, P.cx, P.cy).backproject(*px);
        if (!g1) okj = false;
        else JG.col(k) = (*g1 - G) / h;
      }
      if (!okj) continue;
      const double sigma = std::sqrt((JG * S * JG.transpose()).trace()) * inflate;
      res.accuracy.push_back({rng, sigma});
    }
  }

  // Diagnostics the installer can act on.
  if (in.points.size() < 6) res.warnings.push_back("only " + std::to_string(in.points.size()) +
                                                   " points: 6-8 well spread points (or a few traced lines) make the result robust");
  {
    std::vector<double> ranges;
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& w : P.world) {
      ranges.push_back((w - cam.C.head<2>()).norm());
      mean += w;
    }
    mean /= P.world.size();
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& w : P.world) cov += (w - mean) * (w - mean).transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    const auto [mn, mx] = std::minmax_element(ranges.begin(), ranges.end());
    if (*mx < 1.8 * *mn) res.warnings.push_back("all points are at a similar distance: add points nearer and farther away");
    if (es.eigenvalues()[0] < 0.02 * es.eigenvalues()[1]) res.warnings.push_back("the points lie almost on one line: add points across the road");
  }
  if (!in.focal_px && std::abs(best[6] - P.logf0) > 2.0 * P.logf_sigma)
    res.warnings.push_back("the solved field of view (" + std::to_string(int(std::round(res.fov_deg))) +
                           " deg) does not match the lens setting");
  if (in.height_m && std::abs(cam.C.z() - *in.height_m) > 2.5 * P.h_sigma)
    res.warnings.push_back("the solved height differs from the estimate by " +
                           std::to_string(int(std::round(std::abs(cam.C.z() - *in.height_m)))) + " m");
  {
    std::vector<double> s = ground_err;
    std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
    const double med = s[s.size() / 2];
    for (size_t i = 0; i < ground_err.size(); ++i)
      if (ground_err[i] > std::max(1.0, 3.0 * med)) {
        char w[160];
        std::snprintf(w, sizeof(w), "point %zu is off by %.1f m: check that both clicks show the same spot", i + 1,
                      ground_err[i]);
        res.warnings.push_back(w);
      }
  }

  const bool plausible = all_front && cam.C.z() > 0.5 && cam.C.z() < 150.0;
  res.plausible = plausible;
  res.quality = res.ground_rms_m <= 0.5 ? "good" : res.ground_rms_m <= 1.25 ? "fair" : "poor";
  res.ok = plausible && res.ground_rms_m < 3.0;
  char buf[320];
  std::snprintf(buf, sizeof(buf),
                "f=%.0f px (%.0f deg), pitch=%.2f deg, roll=%.2f deg, height=%.2f m, bearing=%.1f deg, camera %.1f m from "
                "the configured position; points: %.2f m rms on the ground (max %.2f), %.1f px in the image",
                cam.f, res.fov_deg, res.camera.pitch() / kDeg, res.camera.roll() / kDeg, cam.C.z(), bearing,
                cam.C.head<2>().norm(), res.ground_rms_m, res.ground_max_m, res.rms_px);
  res.message = buf;
  if (!plausible) res.message = std::string("no physically plausible camera (") + buf + ")";
  else if (!res.ok) res.message = std::string("points are inconsistent (") + buf + "); check the pairs with the largest error";
  spdlog::info("Manual calibration from {} points, {} lines, {} verticals: {} [{}]", P.img.size(), P.line_px.size(),
               P.verts.size(), res.message, res.quality);
  return res;
}

ManualCalibrationResult calibrateFromCorrespondences(int width, int height, const std::vector<GroundCorrespondence>& pts,
                                                     const GeoAnchor& anchor, std::optional<double> focal_px,
                                                     const std::vector<double>& dist) {
  ManualCalibrationInput in;
  in.width = width;
  in.height = height;
  in.points = pts;
  in.anchor = anchor;
  in.focal_px = focal_px;
  in.dist = dist;
  return calibrateManual(in);
}

}  // namespace jp
