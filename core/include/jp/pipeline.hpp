#pragma once

#include <atomic>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "jp/calib/auto_calibrator.hpp"
#include "jp/calib/refiner.hpp"
#include "jp/health/monitors.hpp"
#include "jp/ingest/video_source.hpp"
#include "jp/output/sink.hpp"
#include "jp/safety/near_miss.hpp"
#include "jp/scene/intersection.hpp"
#include "jp/scene/site_map.hpp"
#include "jp/scene/user_map.hpp"
#include "jp/tracking/tracker.hpp"

namespace jp {

class Detector3D;
class Segmenter;
class Letterbox;
class PreviewServer;

struct PipelineConfig {
  std::string site_id = "site", site_name, camera_id = "cam1";
  VideoSourceConfig source;
  // Camera placement (what the installer knows).
  GeoAnchor anchor;
  double height_m = 7.0, height_sigma = 2.0, bearing_deg = 0.0;
  std::optional<double> focal_px;
  std::vector<double> distortion;
  std::string lens = "unknown";          // lens hint for manual calibration (phone_wide, phone_2x, ...)
  // Calibration.
  std::string calibration_file;
  bool force_recalibration = false;
  bool auto_calibration = true;          // calibration.mode: auto | manual (manual: wait for the web panel)
  double min_safety_confidence = 0.5;    // near-miss events need at least this calibration confidence
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
  std::string backend = "auto";          // auto (GPU via ONNX Runtime CUDA, FP32) | cuda | tensorrt | cpu
  int cpu_threads = 0;
  int imgsz = 0;                         // network input long side (dynamic-size models); 0 = as exported
  int max_det = 300;
  bool crop_to_roi = true;               // run the detector only on the image region of the drawn road area
  float score_threshold = 0.2f;
  double max_range = 250.0;
  // Live settings (changeable from the panel, persisted in state_dir/settings.json).
  std::optional<double> confidence;      // score needed to start a track
  std::optional<double> keypoint_confidence;
  std::vector<std::string> classes;      // reported classes (empty = all)
  bool roi_filter = true;                // ignore objects outside the drawn road area
  // Live view (MJPEG for the panel).
  bool preview = true;
  int preview_port = 8081;
  double preview_fps = 12;
  int preview_width = 960;
  bool preview_overlay = true;
  // Output.
  std::string topic_prefix = "jp";
  std::optional<MqttConfig> mqtt;
  std::string metadata_file;
  double objects_hz = 10, cpm_hz = 10, state_hz = 2, health_hz = 1, occlusion_hz = 1;
  double target_fps = 15;
  std::uint32_t station_id = 1001;
  std::string state_dir = "/data/state";
  // Lock-step start of the cameras of one site replaying recordings (written by the site manager): every camera
  // marks itself ready in `dir` once its models are loaded; when all are, they play from one shared clock.
  struct Sync {
    std::string dir, token;
    std::vector<std::string> cameras;
    double timeout_s = 300;
  };
  std::optional<Sync> sync;

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
  void handleCalibration(const nlohmann::json& cmd, nlohmann::json& out);
  // Live settings (topic config/set): thresholds, range, frame rate, classes, live view.
  nlohmann::json settings() const;
  void applySettings(const nlohmann::json& s, bool persist);
  void publishConfig();
  // Installer-drawn lanes, stop lines, crosswalks and road area (topic map/user/set).
  void setUserMap(const UserMap& m, bool persist);
  void applyUserMap();
  void publishRefine();
  void republishRetained(double stamp);
  void renderPreview(const cv::Mat& frame, const std::vector<ImageDetection>& dets);
  nlohmann::json computeInfo() const;
  // Detector (re)load and the network input region (whole frame or the road area's image box).
  void loadDetector();
  // Waits for the other cameras of the site (cfg_.sync) and returns the shared start time.
  double waitForSync();
  void updateLetterbox();
  cv::Rect computeCrop() const;
  nlohmann::json modelList() const;
  std::string topic(const std::string& leaf) const;
  void emit(const std::string& leaf, const nlohmann::json& body, int qos = 0, bool retain = false);

  PipelineConfig cfg_;
  std::unique_ptr<VideoSource> src_;
  std::unique_ptr<Detector3D> det_;
  std::unique_ptr<Segmenter> seg_;
  std::unique_ptr<Letterbox> lb_;
  std::unique_ptr<MultiSink> sink_;
  std::unique_ptr<AutoCalibrator> calibrator_;
  std::future<CalibrationResult> calib_future_;  // solve() runs off the frame loop (tens of seconds)
  double calib_confidence_ = 0;                 // of the current calibration (manual: 1, auto: solver estimate)
  void cancelCalibration();
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
  std::unique_ptr<PreviewServer> preview_;
  double t_preview_ = -1;
  std::uint64_t loops_seen_ = 0;
  UserMap user_map_;
  nlohmann::json manual_inputs_, manual_result_;  // last manual calibration (inputs, solver report)
  CalibrationRefiner refiner_;
  std::future<std::optional<CalibrationRefiner::Proposal>> refine_future_;
  std::optional<CalibrationRefiner::Proposal> proposal_;
  std::string refine_status_ = "idle";  // idle | collecting | solving | proposal | consistent
  double refine_spread_ = 0, t_refine_pub_ = -1, t_retained_ = -1;
  nlohmann::json calib_msg_;  // last calibration message (republished with the other retained topics)
  std::future<cv::Mat> seg_future_;      // segmentation runs off the frame loop
  std::vector<bool> class_enabled_;
  cv::Rect crop_;
  SignalReader signal_reader_;
  std::vector<SignalState> signals_;
  std::string signals_key_;
  double t_signals_ = -1;
  void updateSignals(const cv::Mat& frame, double stamp);
  std::string models_dir_, model_error_;

  std::uint64_t frames_ = 0, dropped_ = 0;
  std::uint64_t source_frame_ = 0;  // source video frame being processed (see VideoSource::lastIndex)
  double fps_ = 0, latency_ms_ = 0, last_stamp_ = -1;
  double t_objects_ = -1, t_cpm_ = -1, t_state_ = -1, t_health_ = -1, t_occ_ = -1, t_seg_ = -1, t_vis_ = -1,
         t_dyn_ = -1, t_lanes_ = -1, t_drift_ = -1, t_invalid_ = -1;
  bool map_from_file_ = false;
  bool snapshot_sent_ = false;
  std::vector<bool> prev_zone_occ_;
};

}  // namespace jp
