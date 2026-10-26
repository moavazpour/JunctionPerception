#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "jp/calib/auto_calibrator.hpp"
#include "jp/health/monitors.hpp"
#include "jp/ingest/video_source.hpp"
#include "jp/output/sink.hpp"
#include "jp/safety/near_miss.hpp"
#include "jp/scene/intersection.hpp"
#include "jp/scene/site_map.hpp"
#include "jp/tracking/tracker.hpp"

namespace jp {

class Detector3D;
class Segmenter;
class Letterbox;

struct PipelineConfig {
  std::string site_id = "site", site_name, camera_id = "cam1";
  VideoSourceConfig source;
  // Camera placement (what the installer knows).
  GeoAnchor anchor;
  double height_m = 7.0, height_sigma = 2.0, bearing_deg = 0.0;
  std::optional<double> focal_px;
  std::vector<double> distortion;
  // Calibration.
  std::string calibration_file;
  bool force_recalibration = false;
  std::optional<double> road_bearing_deg;
  double calibration_min_s = 45.0;
  bool publish_snapshot = false;
  // Map.
  std::string map_file;
  bool learn_lanes = true;
  std::optional<GeoAnchor> stop_line_point;  // a point on the stop line (lanes get stop_s from it)
  std::vector<std::vector<Eigen::Vector2d>> crosswalks_geo;  // lat/lon polygons
  // Models.
  std::string detector_onnx, segmenter_onnx;
  bool fp16 = true;
  float score_threshold = 0.2f;
  double max_range = 250.0;
  // Output.
  std::string topic_prefix = "jp";
  std::optional<MqttConfig> mqtt;
  std::string metadata_file;
  double objects_hz = 10, cpm_hz = 10, state_hz = 2, health_hz = 1, occlusion_hz = 1;
  double target_fps = 15;
  std::uint32_t station_id = 1001;
  std::string state_dir = "/data/state";

  static PipelineConfig fromYaml(const YAML::Node& n);
};

class Pipeline {
 public:
  explicit Pipeline(PipelineConfig cfg);
  ~Pipeline();
  // Runs until the source ends, max_frames is reached or stop() is called.
  void run(std::uint64_t max_frames = 0);
  void stop() { stop_ = true; }

 private:
  void process(const cv::Mat& frame, double stamp, double capture_wall);
  void onCalibrated(const CalibrationResult* res, const std::string& source);
  std::unique_ptr<AutoCalibrator> makeCalibrator() const;
  void rebuildLanes(double stamp);
  void publishMap();
  void publishSnapshot(const cv::Mat& frame, const std::vector<ImageDetection>& dets);
  // Operator commands (web panel): manual calibration from map correspondences, restart auto-calibration,
  // refresh the calibration snapshot. Received on <prefix>/<site>/<camera>/calibration/manual.
  void handleCommand(const nlohmann::json& cmd);
  std::string topic(const std::string& leaf) const;
  void emit(const std::string& leaf, const nlohmann::json& body, int qos = 0, bool retain = false);

  PipelineConfig cfg_;
  std::unique_ptr<VideoSource> src_;
  std::unique_ptr<Detector3D> det_;
  std::unique_ptr<Segmenter> seg_;
  std::unique_ptr<Letterbox> lb_;
  std::unique_ptr<MultiSink> sink_;
  std::unique_ptr<AutoCalibrator> calibrator_;
  std::optional<CameraModel> camera_;   // calibrated camera
  CameraModel frame_cam_;               // camera corrected for current shake/drift
  Tracker tracker_;
  SiteMap map_;
  LaneLearner learner_;
  std::unique_ptr<IntersectionModel> scene_;
  std::unique_ptr<NearMissDetector> safety_;
  StabilityMonitor stability_;
  VisibilityMonitor visibility_;
  CalibrationMonitor calmon_;
  OcclusionMap occlusion_;
  std::atomic<bool> stop_{false};
  std::mutex cmd_m_;
  std::deque<nlohmann::json> cmds_;

  std::uint64_t frames_ = 0, dropped_ = 0;
  double fps_ = 0, latency_ms_ = 0, last_stamp_ = -1;
  double t_objects_ = -1, t_cpm_ = -1, t_state_ = -1, t_health_ = -1, t_occ_ = -1, t_seg_ = -1, t_vis_ = -1,
         t_dyn_ = -1, t_lanes_ = -1, t_drift_ = -1, t_invalid_ = -1;
  bool map_from_file_ = false;
  bool snapshot_sent_ = false;
  std::vector<bool> prev_zone_occ_;
};

}  // namespace jp
