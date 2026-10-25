#pragma once

#include <Eigen/Core>
#include <deque>
#include <map>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

#include "jp/geometry/camera.hpp"
#include "jp/scene/intersection.hpp"
#include "jp/tracking/tracker.hpp"

namespace jp {

enum class Level : std::uint8_t { Ok = 0, Degraded = 1, Fault = 2 };
std::string_view levelName(Level l);

// ------------------------------------------------------------------ camera stability

// Tracks static background features against a reference frame. Estimates the camera rotation relative
// to the calibration pose every frame (used to stabilise lifting), separates shake from drift.
class StabilityMonitor {
 public:
  struct Config {
    int width = 640;               // processing width
    int max_features = 400;
    double shake_px = 1.2;         // RMS jitter (at processing width) above which we report shaking
    double drift_deg = 0.35;       // persistent offset that counts as "camera moved"
    double drift_hold_s = 5.0;
    double max_compensate_deg = 4.0;
    double min_track_ratio = 0.25;
  };
  struct State {
    std::string status = "init";   // init | stable | shaking | moved | lost
    double jitter_px = 0;          // RMS high-frequency motion (full-resolution pixels)
    double drift_deg = 0;          // low-frequency rotation vs. calibration
    double track_ratio = 1;        // share of reference features still tracked
    Eigen::Matrix3d R_cur_ref = Eigen::Matrix3d::Identity();  // camera rotation vs. reference
    bool compensating = false;
  };

  explicit StabilityMonitor(Config cfg = {}) : cfg_(cfg) {}
  // boxes: current detections (pixels) to mask out moving objects.
  const State& update(double stamp, const cv::Mat& frame_bgr, const cv::Matx33d& K,
                      const std::vector<Eigen::Vector4d>& boxes);
  const State& state() const { return state_; }
  void resetReference() { ref_.release(); }
  // After the calibration absorbed the drift, re-anchor: reference becomes current view.
  void rebase();

 private:
  void setReference(const cv::Mat& gray, const std::vector<Eigen::Vector4d>& boxes, double scale);
  Config cfg_;
  State state_;
  cv::Mat ref_, prev_gray_;
  std::vector<cv::Point2f> ref_pts_;
  double scale_ = 1;
  std::deque<std::pair<double, Eigen::Vector3d>> hist_;  // (t, [rx, ry, rz] deg)
  double drift_since_ = -1, lost_since_ = -1;
  int frames_ = 0;
  Eigen::Matrix3d R_ref_cal_ = Eigen::Matrix3d::Identity();  // reference frame vs. calibration pose
};

// ------------------------------------------------------------------ visibility / weather

class VisibilityMonitor {
 public:
  struct Config {
    double baseline_s = 60.0;          // learning period for the site baseline
    int grid_cols = 8, grid_rows = 6;  // for local obstruction detection
    double obstruction_hold_s = 20.0;
    double frozen_s = 3.0;
  };
  struct State {
    double brightness = 0, contrast = 0, sharpness_ratio = 1, haze = 0, far_contrast_ratio = 1;
    double saturated = 0, dark = 0;
    double obstructed_fraction = 0;
    bool frozen = false;
    std::vector<std::string> conditions;  // night, fog, glare, low_contrast, blur, lens_obstruction, frozen
    double score = 1.0;                   // 0..1 image quality
    double effective_range = 0;           // m, recent detection horizon
    double baseline_range = 0;
    double fragmentation = 0;             // share of short-lived tracks
  };
  explicit VisibilityMonitor(Config cfg = {}) : cfg_(cfg) {}
  const State& update(double stamp, const cv::Mat& frame_bgr, const std::vector<Track>& tracks);
  const State& state() const { return st_; }

 private:
  Config cfg_;
  State st_;
  double t0_ = -1;
  double base_sharp_ = 0, base_far_contrast_ = 0;
  int base_n_ = 0;
  cv::Mat base_cells_, bad_since_;
  cv::Mat prev_small_;
  double still_since_ = -1;
  std::map<std::uint32_t, std::pair<double, double>> first_seen_;  // id -> (time, range at first sight)
  std::deque<std::pair<double, double>> horizon_;                  // (time, first-detection range)
  std::deque<std::pair<double, double>> lifetimes_;                // (end time, lifetime)
  std::map<std::uint32_t, double> last_seen_;
};

// ------------------------------------------------------------------ calibration consistency

class CalibrationMonitor {
 public:
  struct State {
    std::string status = "unknown";  // ok | suspect | invalid | unknown
    double size_ratio = 1;           // median projected-3D / network-2D box height
    double size_slope = 0;           // change of that ratio per 100 m of range
    double heading_error_deg = 0;    // moving vehicles vs. lane direction
    double speed_p95 = 0;
    int samples = 0;
    double score = 1;
    std::vector<std::string> reasons;
  };
  const State& update(double stamp, const CameraModel& cam, const std::vector<Detection3D>& dets,
                      const std::vector<RoadUser>& users, const SiteMap& map);
  const State& state() const { return st_; }
  void reset() { samples_.clear(); }

 private:
  State st_;
  struct S {
    double t, range, ratio;
  };
  std::deque<S> samples_;
  std::deque<std::pair<double, double>> heading_err_;
  std::deque<std::pair<double, double>> speeds_;
};

// ------------------------------------------------------------------ occlusion

class OcclusionMap {
 public:
  enum Cell : uint8_t { Visible = 0, OutOfView = 1, StaticOccluded = 2, DynamicOccluded = 3, BeyondResolution = 4 };
  struct Config {
    double cell = 1.0;
    double min_ped_px = 14.0;   // pedestrian must be at least this tall to count as observable
    double probe_height = 0.9;  // m, height of the point that must be visible (pedestrian torso)
  };
  struct Segment {
    std::string zone;     // lane or crosswalk id
    double s0 = 0, s1 = 0;
    std::string cause;    // static | vehicle:<id> | out_of_view | resolution
  };
  struct ZoneCoverage {
    std::string zone;
    double visible = 1, static_occluded = 0, dynamic_occluded = 0, unobservable = 0;
  };

  explicit OcclusionMap(Config cfg = {}) : cfg_(cfg) {}
  void configure(const CameraModel& cam, const SiteMap& map, double default_range = 150.0);
  // Static occlusion is learned from segmentation (low rate); dynamic occlusion from current tracks.
  void updateStatic(const cv::Mat& labels, const CameraModel& cam);
  void updateDynamic(const CameraModel& cam, const std::vector<Track>& tracks);
  bool occluded(const Eigen::Vector2d& p) const;
  const std::vector<ZoneCoverage>& coverage() const { return coverage_; }
  const std::vector<Segment>& segments() const { return segments_; }
  // Grid for visualisation: origin, size, cell, row-major states.
  Eigen::Vector2d origin() const { return origin_; }
  int cols() const { return cols_; }
  int rows() const { return rows_; }
  double cellSize() const { return cfg_.cell; }
  const std::vector<uint8_t>& grid() const { return state_; }

 private:
  void summarize(const SiteMap& map);
  Config cfg_;
  Eigen::Vector2d origin_{0, 0};
  int cols_ = 0, rows_ = 0;
  std::vector<uint8_t> base_;          // OutOfView / BeyondResolution / Visible from geometry
  std::vector<float> static_prob_;     // EMA probability the ground point is hidden by a static object
  std::vector<uint8_t> state_;
  std::vector<std::uint32_t> occluder_;
  std::vector<ZoneCoverage> coverage_;
  std::vector<Segment> segments_;
  const SiteMap* map_ = nullptr;
  std::vector<Eigen::Vector2d> roi_;
};

// ------------------------------------------------------------------ integrity

struct HealthReport {
  Level level = Level::Ok;
  double confidence = 1.0;
  std::vector<std::string> reasons;
  double fps = 0, latency_ms = 0, detector_ms = 0;
  std::uint64_t frames = 0, dropped = 0;
  StabilityMonitor::State stability;
  VisibilityMonitor::State visibility;
  CalibrationMonitor::State calibration;
  std::vector<OcclusionMap::ZoneCoverage> coverage;
  std::vector<OcclusionMap::Segment> occluded_segments;
  bool calibrated = false;
};

HealthReport assessIntegrity(const StabilityMonitor::State& stab, const VisibilityMonitor::State& vis,
                             const CalibrationMonitor::State& cal, const OcclusionMap& occ, double fps,
                             double target_fps, double latency_ms, double detector_ms, bool calibrated);

}  // namespace jp
