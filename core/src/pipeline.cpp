#include "jp/pipeline.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "jp/calib/manual_calibration.hpp"
#include "jp/infer/detector.hpp"
#include "jp/infer/engine.hpp"
#include "jp/output/preview_server.hpp"
#include "jp/output/messages.hpp"

namespace jp {
namespace {
double wallNow() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string base64(const std::vector<uchar>& in) {
  static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((in.size() + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 2 < in.size(); i += 3) {
    const unsigned v = (in[i] << 16) | (in[i + 1] << 8) | in[i + 2];
    out += {tbl[v >> 18], tbl[(v >> 12) & 63], tbl[(v >> 6) & 63], tbl[v & 63]};
  }
  if (i < in.size()) {
    const unsigned v = (in[i] << 16) | (i + 1 < in.size() ? in[i + 1] << 8 : 0);
    out += {tbl[v >> 18], tbl[(v >> 12) & 63], i + 1 < in.size() ? tbl[(v >> 6) & 63] : '=', '='};
  }
  return out;
}

template <typename T>
std::optional<T> opt(const YAML::Node& n) {
  if (n && !n.IsNull()) return n.as<T>();
  return std::nullopt;
}
}  // namespace

PipelineConfig PipelineConfig::fromYaml(const YAML::Node& n) {
  PipelineConfig c;
  if (auto s = n["site"]) {
    c.site_id = s["id"].as<std::string>(c.site_id);
    c.site_name = s["name"].as<std::string>("");
  }
  const auto cam = n["camera"];
  c.camera_id = cam["id"].as<std::string>(c.camera_id);
  c.source.uri = cam["source"].as<std::string>();
  c.source.realtime = cam["realtime"].as<bool>(true);
  c.source.loop = cam["loop"].as<bool>(false);
  c.source.max_fps = cam["max_fps"].as<double>(0);
  if (auto p = cam["position"]) {
    c.anchor = {p["lat"].as<double>(), p["lon"].as<double>(), p["alt"].as<double>(0.0), true};
    c.height_m = p["height_m"].as<double>(c.height_m);
    c.height_sigma = p["height_sigma"].as<double>(c.height_sigma);
    c.bearing_deg = p["bearing_deg"].as<double>(c.bearing_deg);
  }
  c.lens = cam["lens"].as<std::string>(c.lens);
  if (auto i = cam["intrinsics"]) {
    c.focal_px = opt<double>(i["focal_px"]);
    if (i["distortion"]) c.distortion = i["distortion"].as<std::vector<double>>();
  }
  if (auto k = n["calibration"]) {
    c.calibration_file = k["file"].as<std::string>("");
    c.force_recalibration = k["force"].as<bool>(false);
    c.auto_calibration = k["mode"].as<std::string>("auto") != "manual";
    c.min_safety_confidence = k["min_safety_confidence"].as<double>(c.min_safety_confidence);
    c.road_bearing_deg = opt<double>(k["road_bearing_deg"]);
    c.calibration_min_s = k["min_duration_s"].as<double>(c.calibration_min_s);
    c.publish_snapshot = k["publish_snapshot"].as<bool>(false);
  }
  if (auto m = n["map"]) {
    c.map_file = m["file"].as<std::string>("");
    c.learn_lanes = m["learn_lanes"].as<bool>(true);
    c.roi_filter = m["roi_filter"].as<bool>(c.roi_filter);
    if (auto s = m["stop_line"]) c.stop_line_point = GeoAnchor{s["lat"].as<double>(), s["lon"].as<double>(), 0, true};
    if (auto cw = m["crosswalks"]) {
      for (const auto& poly : cw) {
        std::vector<Eigen::Vector2d> pts;
        for (const auto& p : poly) pts.emplace_back(p[0].as<double>(), p[1].as<double>());
        c.crosswalks_geo.push_back(pts);
      }
    }
  }
  const auto md = n["models"];
  c.detector_onnx = md["detector"].as<std::string>();
  c.segmenter_onnx = md["segmentation"].as<std::string>("");
  c.fp16 = md["fp16"].as<bool>(true);
  c.score_threshold = md["score_threshold"].as<float>(c.score_threshold);
  c.max_range = md["max_range_m"].as<double>(c.max_range);
  c.backend = md["backend"].as<std::string>(c.backend);
  c.cpu_threads = md["cpu_threads"].as<int>(c.cpu_threads);
  c.confidence = opt<double>(md["confidence"]);
  c.keypoint_confidence = opt<double>(md["keypoint_confidence"]);
  if (md["classes"]) c.classes = md["classes"].as<std::vector<std::string>>();
  if (auto p = n["preview"]) {
    c.preview = p["enabled"].as<bool>(c.preview);
    c.preview_port = p["port"].as<int>(c.preview_port);
    c.preview_fps = p["fps"].as<double>(c.preview_fps);
    c.preview_width = p["width"].as<int>(c.preview_width);
    c.preview_overlay = p["overlay"].as<bool>(c.preview_overlay);
  }
  if (auto o = n["output"]) {
    c.topic_prefix = o["topic_prefix"].as<std::string>(c.topic_prefix);
    if (auto q = o["mqtt"]) {
      MqttConfig mc;
      mc.uri = q["uri"].as<std::string>(mc.uri);
      mc.client_id = q["client_id"].as<std::string>(c.site_id + "-" + c.camera_id);
      mc.username = q["username"].as<std::string>("");
      mc.password = q["password"].as<std::string>("");
      c.mqtt = mc;
    }
    c.metadata_file = o["file"].as<std::string>("");
    if (auto r = o["rates_hz"]) {
      c.objects_hz = r["objects"].as<double>(c.objects_hz);
      c.cpm_hz = r["cpm"].as<double>(c.cpm_hz);
      c.state_hz = r["state"].as<double>(c.state_hz);
      c.health_hz = r["health"].as<double>(c.health_hz);
      c.occlusion_hz = r["occlusion"].as<double>(c.occlusion_hz);
    }
    c.station_id = o["station_id"].as<std::uint32_t>(c.station_id);
  }
  if (auto h = n["health"]) c.target_fps = h["target_fps"].as<double>(c.target_fps);
  c.state_dir = n["state_dir"].as<std::string>(c.state_dir);
  return c;
}

Pipeline::Pipeline(PipelineConfig cfg) : cfg_(std::move(cfg)) {
  std::filesystem::create_directories(cfg_.state_dir);
  src_ = std::make_unique<VideoSource>(cfg_.source);
  DetectorConfig dc;
  dc.onnx = cfg_.detector_onnx;
  dc.fp16 = cfg_.fp16;
  dc.score_threshold = std::min(cfg_.score_threshold, 0.15f);
  dc.backend = cfg_.backend;
  dc.threads = cfg_.cpu_threads;
  det_ = std::make_unique<Detector3D>(dc);
  if (!cfg_.segmenter_onnx.empty()) {
    // The segmenter follows the detector's backend (a CPU detector means there is no usable GPU).
    EngineOptions so;
    so.backend = det_->engine().backend();
    so.fp16 = cfg_.fp16;
    so.threads = cfg_.cpu_threads;
    try {
      seg_ = std::make_unique<Segmenter>(cfg_.segmenter_onnx, so);
    } catch (const std::exception& e) {
      spdlog::warn("Segmentation disabled: {}", e.what());
    }
  }
  sink_ = std::make_unique<MultiSink>();
  if (cfg_.mqtt) sink_->add(std::make_unique<MqttSink>(*cfg_.mqtt));
  if (!cfg_.metadata_file.empty()) sink_->add(std::make_unique<FileSink>(cfg_.metadata_file));
  for (const std::string leaf : {"calibration/manual", "map/user/set", "config/set"}) {
    sink_->subscribe(topic(leaf), [this, leaf](const std::string&, const std::string& payload) {
      auto j = nlohmann::json::parse(payload, nullptr, false);
      if (j.is_discarded() || !j.is_object()) return;
      j["_topic"] = leaf;
      std::lock_guard<std::mutex> lk(cmd_m_);
      cmds_.push_back(std::move(j));
    });
  }
  if (cfg_.preview) preview_ = std::make_unique<PreviewServer>(cfg_.preview_port);
  // Persisted operator state: live settings, drawn map, last manual calibration inputs.
  const std::filesystem::path sd(cfg_.state_dir);
  {
    class_enabled_.assign(kClassNames.size(), true);
    nlohmann::json s = settings();
    if (cfg_.confidence) s["confidence"] = *cfg_.confidence;
    if (cfg_.keypoint_confidence) s["keypoint_confidence"] = *cfg_.keypoint_confidence;
    if (!cfg_.classes.empty()) s["classes"] = cfg_.classes;
    if (std::filesystem::exists(sd / "settings.json")) {
      std::ifstream f(sd / "settings.json");
      const auto j = nlohmann::json::parse(f, nullptr, false);
      if (j.is_object()) s.update(j);
    }
    applySettings(s, false);
  }
  if (std::filesystem::exists(sd / "user_map.json")) {
    std::ifstream f(sd / "user_map.json");
    const auto j = nlohmann::json::parse(f, nullptr, false);
    if (j.is_object()) user_map_ = UserMap::fromJson(j);
  }
  if (std::filesystem::exists(sd / "calibration_inputs.json")) {
    std::ifstream f(sd / "calibration_inputs.json");
    manual_inputs_ = nlohmann::json::parse(f, nullptr, false);
    if (manual_inputs_.is_discarded()) manual_inputs_ = nullptr;
  }

  const cv::Size fs = src_->size();
  cv::Matx33d K = cv::Matx33d::eye();
  if (cfg_.focal_px) K = {*cfg_.focal_px, 0, (fs.width - 1) / 2.0, 0, *cfg_.focal_px, (fs.height - 1) / 2.0, 0, 0, 1};
  lb_ = std::make_unique<Letterbox>(fs, det_->inputSize(), K, cfg_.distortion);

  if (!cfg_.map_file.empty() && std::filesystem::exists(cfg_.map_file)) {
    std::ifstream f(cfg_.map_file);
    map_ = SiteMap::fromJson(nlohmann::json::parse(f));
    map_from_file_ = true;
    spdlog::info("Loaded site map with {} lanes from {}", map_.lanes.size(), cfg_.map_file);
  }
  map_.name = cfg_.site_name;
  scene_ = std::make_unique<IntersectionModel>(map_, nullptr);
  safety_ = std::make_unique<NearMissDetector>();

  calibrator_ = makeCalibrator();

  if (!cfg_.force_recalibration && !cfg_.calibration_file.empty() && std::filesystem::exists(cfg_.calibration_file)) {
    std::ifstream f(cfg_.calibration_file);
    const auto j = nlohmann::json::parse(f);
    CameraModel cam = CameraModel::fromJson(j);
    if (cam.width() == fs.width && cam.height() == fs.height) {
      camera_ = cam;
      // A stored manual calibration is trusted; a stored auto-calibration keeps its own estimate.
      calib_confidence_ = j.value("source", "").rfind("manual", 0) == 0 ? 1.0 : j.value("confidence", 0.0);
      if (j.contains("manual")) manual_result_ = j["manual"];
      onCalibrated(nullptr, "file");
    } else {
      spdlog::warn("Stored calibration is for a different resolution; recalibrating");
    }
  }
  publishConfig();
  {
    auto h = msg::header("user_map", cfg_.site_id, cfg_.camera_id, 0);
    h["map"] = user_map_.toJson();
    emit("map/user", h, 1, true);
  }
  if (manual_inputs_.is_object()) {
    auto h = msg::header("calibration_inputs", cfg_.site_id, cfg_.camera_id, 0);
    h["inputs"] = manual_inputs_;
    emit("calibration/inputs", h, 1, true);
  }
  publishRefine();
}

Pipeline::~Pipeline() = default;

std::unique_ptr<AutoCalibrator> Pipeline::makeCalibrator() const {
  CalibrationPriors pri;
  pri.height_m = cfg_.height_m;
  pri.height_sigma = cfg_.height_sigma;
  pri.focal_px = cfg_.focal_px;
  pri.road_bearing_deg = cfg_.road_bearing_deg;
  pri.camera_bearing_deg = cfg_.bearing_deg;
  pri.anchor = cfg_.anchor;
  AutoCalibrator::Config cc;
  cc.min_duration_s = cfg_.calibration_min_s;
  return std::make_unique<AutoCalibrator>(src_->size().width, src_->size().height, pri, cc);
}

std::string Pipeline::topic(const std::string& leaf) const {
  return cfg_.topic_prefix + "/" + cfg_.site_id + "/" + cfg_.camera_id + "/" + leaf;
}

void Pipeline::emit(const std::string& leaf, const nlohmann::json& body, int qos, bool retain) {
  sink_->publish(topic(leaf), body.dump(), qos, retain);
}

void Pipeline::onCalibrated(const CalibrationResult* res, const std::string& source) {
  CameraModel& cam = *camera_;
  if (!cam.anchor.valid) cam.anchor = cfg_.anchor;
  frame_cam_ = cam;
  scene_->setCamera(&frame_cam_);
  safety_->setCamera(&frame_cam_);
  stability_.rebase();
  calmon_.reset();
  tracker_.reset();
  // Road axis for lane learning: the vanishing direction (or the camera's forward direction).
  double axis = M_PI / 2 - cam.bearingDeg() * M_PI / 180.0;
  if (res && res->road_yaw) {
    axis = *res->road_yaw;
  } else if (res) {
    const Eigen::Vector3d d = (cam.K().inverse() *
                               Eigen::Vector3d(res->vanishing_point.x(), res->vanishing_point.y(), 1)).normalized();
    Eigen::Vector3d g = cam.R_ground_cam() * d;
    axis = std::atan2(g.y(), g.x());
  }
  bool ahead = true;
  if (cfg_.stop_line_point) {
    const Eigen::Vector2d p = geoToEnu(cam.anchor, cfg_.stop_line_point->lat, cfg_.stop_line_point->lon);
    ahead = p.dot(Eigen::Vector2d(std::cos(axis), std::sin(axis))) > 0;
  }
  // Orient the axis towards the junction.
  if (!ahead) axis += M_PI;
  learner_.setAxis(axis, true);
  if (!map_from_file_) {
    map_.crosswalks.clear();
    for (const auto& poly : cfg_.crosswalks_geo) {
      std::vector<Eigen::Vector2d> pts;
      for (const auto& p : poly) pts.push_back(geoToEnu(cam.anchor, p.x(), p.y()));
      map_.crosswalks.push_back(pts);
    }
  }
  if (!user_map_.empty()) {
    const SiteMap um = user_map_.build(cam.anchor, cfg_.site_name);
    if (!um.lanes.empty()) {
      map_.lanes = um.lanes;
      map_.zones = um.zones;
    }
    if (!um.crosswalks.empty()) map_.crosswalks = um.crosswalks;
    map_.roi = um.roi;
  }
  // Traffic check of a manual calibration (proposes a correction, the operator decides).
  if (!cfg_.auto_calibration && source != "drift-compensated") {
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> pts;
    if (manual_inputs_.is_object())
      for (const auto& p : manual_inputs_.value("points", nlohmann::json::array()))
        pts.emplace_back(Eigen::Vector2d(p.value("u", 0.0), p.value("v", 0.0)),
                         geoToEnu(cam.anchor, p.value("lat", 0.0), p.value("lon", 0.0)));
    if (refine_future_.valid()) refine_future_.wait(), refine_future_ = {};
    refiner_.reset(cam, pts, user_map_.lanes.empty() ? std::vector<Lane>{} : map_.lanes);
    proposal_.reset();
    refine_status_ = "collecting";
    publishRefine();
  }
  scene_->setMap(map_);
  occlusion_.configure(cam, map_, cfg_.max_range);
  if (res) calib_confidence_ = res->confidence;
  else if (source == "manual") calib_confidence_ = 1.0;  // "file" and "drift-compensated" keep the current value
  nlohmann::json cj = msg::calibration(cam, res, source);
  cj["confidence"] = calib_confidence_;
  if (source.rfind("manual", 0) == 0 || (source == "file" && !manual_result_.is_null())) cj["manual"] = manual_result_;
  cj["road_axis_yaw"] = axis;
  if (!cfg_.calibration_file.empty() && source != "file") {
    std::ofstream(cfg_.calibration_file) << cj.dump(1);
  }
  calib_msg_ = msg::header("calibration", cfg_.site_id, cfg_.camera_id, last_stamp_);
  calib_msg_["calibration"] = cj;
  emit("calibration", calib_msg_, 1, true);
  publishMap();
  snapshot_sent_ = false;
}

void Pipeline::publishMap() {
  if (!camera_) return;
  auto h = msg::header("map", cfg_.site_id, cfg_.camera_id, last_stamp_);
  h["map"] = map_.toJson();
  h["camera_model"] = camera_->toJson();
  emit("map", h, 1, true);
  if (!cfg_.state_dir.empty()) std::ofstream(cfg_.state_dir + "/site_map_" + cfg_.camera_id + ".json") << map_.toJson().dump(1);
}

void Pipeline::publishSnapshot(const cv::Mat& frame, const std::vector<ImageDetection>& dets) {
  // Calibration view only: downscaled, every detected road user pixelated beyond recognition.
  cv::Mat img;
  const double s = 960.0 / frame.cols;
  cv::resize(frame, img, cv::Size(), s, s, cv::INTER_AREA);
  for (const auto& d : dets) {
    cv::Rect r(cv::Point(static_cast<int>(d.box2d[0] * s) - 4, static_cast<int>(d.box2d[1] * s) - 4),
               cv::Point(static_cast<int>(d.box2d[2] * s) + 4, static_cast<int>(d.box2d[3] * s) + 4));
    r &= cv::Rect(0, 0, img.cols, img.rows);
    if (r.area() <= 0) continue;
    cv::Mat roi = img(r), tiny;
    cv::resize(roi, tiny, cv::Size(std::max(1, r.width / 12), std::max(1, r.height / 12)), 0, 0, cv::INTER_AREA);
    cv::resize(tiny, roi, r.size(), 0, 0, cv::INTER_NEAREST);
  }
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
  std::vector<uchar> buf;
  cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, 70});
  auto h = msg::header("snapshot", cfg_.site_id, cfg_.camera_id, last_stamp_);
  h["width"] = img.cols;
  h["height"] = img.rows;
  h["scale"] = s;
  h["frame_width"] = frame.cols;
  h["frame_height"] = frame.rows;
  const GeoAnchor& a = camera_ && camera_->anchor.valid ? camera_->anchor : cfg_.anchor;
  if (a.valid) h["anchor"] = {{"lat", a.lat}, {"lon", a.lon}};
  h["jpeg_base64"] = base64(buf);
  h["note"] = "calibration view; road users pixelated";
  emit("snapshot", h, 1, true);
  snapshot_sent_ = true;
}

void Pipeline::rebuildLanes(double stamp) {
  if (map_from_file_ || !cfg_.learn_lanes || !camera_ || !user_map_.lanes.empty()) return;
  auto lanes = learner_.build();
  if (lanes.empty()) return;
  // Extend inbound lanes to the configured stop line and set stop_s from it (unless learned).
  if (cfg_.stop_line_point) {
    const Eigen::Vector2d p = geoToEnu(camera_->anchor, cfg_.stop_line_point->lat, cfg_.stop_line_point->lon);
    for (auto& l : lanes) {
      auto& pts = l.centerline.pts;
      if (l.to_junction) {
        const Eigen::Vector2d dir = (pts.back() - pts[pts.size() - 2]).normalized();
        const double beyond = (p - pts.back()).dot(dir);
        if (beyond > 0) pts.push_back(pts.back() + dir * (beyond + 3.0));
        if (!l.stop_s) l.stop_s = l.centerline.project(p).s;
      } else {
        const Eigen::Vector2d dir = (pts[1] - pts[0]).normalized();
        const double before = (pts[0] - p).dot(dir);
        if (before > 0) pts.insert(pts.begin(), pts[0] - dir * (before + 3.0));
      }
    }
  }
  map_.lanes = lanes;
  map_.zones.clear();
  map_.addDefaultDetectors();
  scene_->setMap(map_);
  occlusion_.configure(*camera_, map_, cfg_.max_range);
  publishMap();
  spdlog::info("Lane model updated: {} lanes from {} samples", lanes.size(), learner_.samples());
  (void)stamp;
}

void Pipeline::cancelCalibration() {
  // The solver reads the calibrator: let a running solve finish before the calibrator is replaced.
  if (calib_future_.valid()) {
    calib_future_.wait();
    calib_future_ = {};
  }
}

void Pipeline::handleCommand(const nlohmann::json& cmd) {
  const std::string from = cmd.value("_topic", "calibration/manual");
  if (from == "config/set") {
    applySettings(cmd.value("settings", nlohmann::json::object()), true);
    publishConfig();
    return;
  }
  if (from == "map/user/set") {
    setUserMap(UserMap::fromJson(cmd.value("map", nlohmann::json::object())), true);
    return;
  }
  nlohmann::json out = {{"action", cmd.value("action", "solve")}};
  handleCalibration(cmd, out);
  auto h = msg::header("calibration_command", cfg_.site_id, cfg_.camera_id, last_stamp_);
  h["result"] = out;
  emit("calibration/manual/result", h, 1);
}

void Pipeline::handleCalibration(const nlohmann::json& cmd, nlohmann::json& out) {
  const std::string action = out["action"];
  using J = nlohmann::json;
  if (action == "snapshot") {
    snapshot_sent_ = false;
    out["ok"] = true;
  } else if (action == "auto") {
    cancelCalibration();
    camera_.reset();
    calibrator_ = makeCalibrator();
    if (!cfg_.calibration_file.empty()) std::filesystem::remove(cfg_.calibration_file);
    snapshot_sent_ = false;
    manual_result_ = nullptr;
    refiner_ = CalibrationRefiner();
    proposal_.reset();
    refine_status_ = "idle";
    publishRefine();
    // Clear the retained calibration and map so viewers do not keep showing the discarded geometry.
    calib_msg_ = nullptr;
    sink_->publish(topic("calibration"), "", 1, true);
    sink_->publish(topic("map"), "", 1, true);
    sink_->publish(topic("detectors"), "", 1, true);
    for (const auto& z : scene_->zones()) sink_->publish(topic("detector/" + z.id), "", 1, true);
    out["ok"] = true;
    out["message"] = cfg_.auto_calibration ? "calibration cleared; auto-calibration restarted"
                                           : "calibration cleared; waiting for a manual calibration";
    spdlog::info("Operator cleared the calibration");
  } else if (action == "solve") {
    ManualCalibrationInput in;
    in.width = src_->size().width;
    in.height = src_->size().height;
    in.anchor = cfg_.anchor;
    in.focal_px = cfg_.focal_px;
    in.dist = cfg_.distortion;
    auto px = [](const J& p) { return Eigen::Vector2d(p.at(0).get<double>(), p.at(1).get<double>()); };
    for (const auto& p : cmd.value("points", J::array()))
      in.points.push_back({{p.at("u").get<double>(), p.at("v").get<double>()}, p.at("lat").get<double>(), p.at("lon").get<double>()});
    for (const auto& l : cmd.value("lines", J::array())) {
      LineCorrespondence lc;
      for (const auto& q : l.at("px")) lc.px.push_back(px(q));
      const auto& g = l.at("geo");
      if (g.size() < 2 || lc.px.size() < 2) continue;
      lc.geo = {px(g.at(0)), px(g.at(1))};
      in.lines.push_back(lc);
    }
    for (const auto& v : cmd.value("verticals", J::array()))
      if (v.size() >= 2) in.verticals.push_back({px(v.at(0)), px(v.at(1))});
    in.height_m = cmd.contains("height_m") && cmd["height_m"].is_number() ? cmd["height_m"].get<double>() : cfg_.height_m;
    in.height_sigma = cmd.value("height_sigma", cfg_.height_sigma);
    in.lens = cmd.value("lens", cfg_.lens);
    const auto r = calibrateManual(in);
    J rep = {{"ok", r.ok},
             {"plausible", r.plausible},
             {"message", r.message},
             {"quality", r.quality},
             {"fit", r.fit},
             {"rms_px", r.rms_px},
             {"max_px", r.max_px},
             {"ground_rms_m", r.ground_rms_m},
             {"ground_max_m", r.ground_max_m},
             {"focal_px", r.focal_px},
             {"fov_deg", r.fov_deg},
             {"warnings", r.warnings},
             {"residuals", J::array()},
             {"accuracy", J::array()}};
    for (const auto& e : r.residuals) rep["residuals"].push_back({{"kind", e.kind}, {"index", e.index}, {"px", e.px}, {"m", e.m}, {"norm", e.norm}});
    for (const auto& a : r.accuracy) rep["accuracy"].push_back({{"range_m", a.range_m}, {"sigma_m", a.sigma_m}});
    if (r.plausible) {
      rep["pitch_deg"] = r.camera.pitch() * 180 / M_PI;
      rep["roll_deg"] = r.camera.roll() * 180 / M_PI;
      rep["height_m"] = r.camera.cameraHeight();
      rep["bearing_deg"] = r.camera.bearingDeg();
    }
    out.update(rep);
    // The solved camera even when not applied: the panel previews its fit before the operator applies it.
    if (r.plausible) out["camera"] = r.camera.toJson();
    manual_inputs_ = {{"points", cmd.value("points", J::array())},
                      {"lines", cmd.value("lines", J::array())},
                      {"verticals", cmd.value("verticals", J::array())},
                      {"height_m", in.height_m ? J(*in.height_m) : J()},
                      {"height_sigma", in.height_sigma},
                      {"lens", in.lens}};
    if (!cfg_.state_dir.empty()) std::ofstream(cfg_.state_dir + "/calibration_inputs.json") << manual_inputs_.dump(1);
    {
      auto h = msg::header("calibration_inputs", cfg_.site_id, cfg_.camera_id, last_stamp_);
      h["inputs"] = manual_inputs_;
      emit("calibration/inputs", h, 1, true);
    }
    const bool apply = cmd.value("apply", true) && (r.ok || (cmd.value("force", false) && r.plausible));
    out["applied"] = apply;
    if (apply) {
      manual_result_ = rep;
      cancelCalibration();
      camera_ = r.camera;
      onCalibrated(nullptr, "manual");
    }
  } else if (action == "refine_accept") {
    if (proposal_ && camera_) {
      const auto p = *proposal_;
      camera_ = p.camera;
      if (manual_result_.is_object())
        manual_result_["refined"] = {{"spread_before_deg", p.spread_before_deg}, {"spread_after_deg", p.spread_after_deg},
                                     {"d_pitch_deg", p.d_pitch_deg}, {"d_roll_deg", p.d_roll_deg},
                                     {"d_yaw_deg", p.d_yaw_deg}, {"d_focal_rel", p.d_focal_rel}};
      proposal_.reset();
      onCalibrated(nullptr, "manual-refined");
      out["ok"] = true;
      out["message"] = "correction applied";
    } else {
      out["ok"] = false;
      out["message"] = "no correction proposed";
    }
  } else if (action == "refine_reject") {
    proposal_.reset();
    refine_status_ = "rejected";
    publishRefine();
    out["ok"] = true;
  } else {
    out["ok"] = false;
    out["message"] = "unknown action";
  }
}

nlohmann::json Pipeline::settings() const {
  nlohmann::json classes = nlohmann::json::array();
  for (size_t i = 0; i < kClassNames.size(); ++i)
    if (class_enabled_.size() != kClassNames.size() || class_enabled_[i]) classes.push_back(std::string(kClassNames[i]));
  return {{"confidence", tracker_.config().new_track_score},
          {"keypoint_confidence", CameraModel::keypointConfidence()},
          {"max_range_m", cfg_.max_range},
          {"max_fps", src_->maxFps()},
          {"classes", classes},
          {"roi_filter", cfg_.roi_filter},
          {"preview_overlay", cfg_.preview_overlay},
          {"preview_fps", cfg_.preview_fps}};
}

void Pipeline::applySettings(const nlohmann::json& s, bool persist) {
  auto num = [&](const char* k, double lo, double hi) -> std::optional<double> {
    if (!s.contains(k) || !s[k].is_number()) return std::nullopt;
    return std::clamp(s[k].get<double>(), lo, hi);
  };
  if (auto v = num("confidence", 0.05, 0.95)) {
    auto& tc = tracker_.config();
    tc.new_track_score = static_cast<float>(*v);
    tc.high_score = static_cast<float>(std::max(0.1, *v - 0.05));
    det_->setScoreThreshold(static_cast<float>(std::min(0.15, *v)));
  }
  if (auto v = num("keypoint_confidence", 0.0, 1.0)) CameraModel::setKeypointConfidence(*v);
  if (auto v = num("max_range_m", 20, 1000)) cfg_.max_range = *v;
  if (auto v = num("max_fps", 0, 60)) src_->setMaxFps(*v);
  if (auto v = num("preview_fps", 1, 30)) cfg_.preview_fps = *v;
  if (s.contains("roi_filter") && s["roi_filter"].is_boolean()) cfg_.roi_filter = s["roi_filter"];
  if (s.contains("preview_overlay") && s["preview_overlay"].is_boolean()) cfg_.preview_overlay = s["preview_overlay"];
  if (class_enabled_.size() != kClassNames.size()) class_enabled_.assign(kClassNames.size(), true);
  if (s.contains("classes") && s["classes"].is_array()) {
    class_enabled_.assign(kClassNames.size(), false);
    for (const auto& c : s["classes"])
      for (size_t i = 0; i < kClassNames.size(); ++i)
        if (c.is_string() && c.get<std::string>() == kClassNames[i]) class_enabled_[i] = true;
  }
  if (persist) {
    if (!cfg_.state_dir.empty()) std::ofstream(cfg_.state_dir + "/settings.json") << settings().dump(1);
    spdlog::info("Settings changed: {}", s.dump());
  }
}

nlohmann::json Pipeline::computeInfo() const {
  const auto& e = det_->engine();
  return {{"backend", e.backend()},
          {"device", e.device()},
          {"precision", e.precision()},
          {"model", std::filesystem::path(cfg_.detector_onnx).filename().string()},
          {"input", {det_->inputSize().width, det_->inputSize().height}},
          {"available", availableBackends()},
          {"segmentation", seg_ != nullptr}};
}

void Pipeline::publishConfig() {
  auto h = msg::header("config", cfg_.site_id, cfg_.camera_id, last_stamp_);
  h["site_name"] = cfg_.site_name;
  h["settings"] = settings();
  h["compute"] = computeInfo();
  h["source"] = {{"uri", src_->describe()},
                 {"live", !src_->isFile()},
                 {"loop", src_->isFile() && cfg_.source.loop},
                 {"fps", src_->fps()},
                 {"width", src_->size().width},
                 {"height", src_->size().height}};
  h["placement"] = {{"lat", cfg_.anchor.lat},   {"lon", cfg_.anchor.lon},        {"height_m", cfg_.height_m},
                 {"height_sigma", cfg_.height_sigma}, {"bearing_deg", cfg_.bearing_deg}, {"lens", cfg_.lens}};
  h["preview"] = {{"enabled", preview_ != nullptr}, {"port", cfg_.preview_port}};
  h["calibration_mode"] = cfg_.auto_calibration ? "auto" : "manual";
  emit("config", h, 1, true);
}

void Pipeline::setUserMap(const UserMap& m, bool persist) {
  user_map_ = m;
  if (persist && !cfg_.state_dir.empty()) std::ofstream(cfg_.state_dir + "/user_map.json") << user_map_.toJson().dump(1);
  auto h = msg::header("user_map", cfg_.site_id, cfg_.camera_id, last_stamp_);
  h["map"] = user_map_.toJson();
  emit("map/user", h, 1, true);
  spdlog::info("Site layout: {} lanes, {} stop lines, {} crosswalks, road area {}", m.lanes.size(), m.stop_lines.size(),
               m.crosswalks.size(), m.roi.empty() ? "none" : "set");
  applyUserMap();
}

void Pipeline::applyUserMap() {
  if (!camera_) return;
  const SiteMap um = user_map_.build(camera_->anchor, cfg_.site_name);
  if (!um.lanes.empty()) {
    map_.lanes = um.lanes;
    map_.zones = um.zones;
  } else if (std::any_of(map_.lanes.begin(), map_.lanes.end(), [](const Lane& l) { return l.source == "user"; })) {
    map_.lanes.clear();  // drawn lanes removed: learn them from traffic again
    map_.zones.clear();
  }
  if (!um.crosswalks.empty()) map_.crosswalks = um.crosswalks;
  map_.roi = um.roi;
  scene_->setMap(map_);
  occlusion_.configure(*camera_, map_, cfg_.max_range);
  refiner_.setLanes(um.lanes.empty() ? std::vector<Lane>{} : map_.lanes);
  publishMap();
}

// The broker keeps no state across restarts (persistence off): retained topics are refreshed every minute.
void Pipeline::republishRetained(double stamp) {
  if (t_retained_ >= 0 && stamp - t_retained_ < 60.0) return;
  const bool first = t_retained_ < 0;
  t_retained_ = stamp;
  if (first) return;  // everything was just published at start-up
  publishConfig();
  auto h = msg::header("user_map", cfg_.site_id, cfg_.camera_id, stamp);
  h["map"] = user_map_.toJson();
  emit("map/user", h, 1, true);
  if (manual_inputs_.is_object()) {
    auto i = msg::header("calibration_inputs", cfg_.site_id, cfg_.camera_id, stamp);
    i["inputs"] = manual_inputs_;
    emit("calibration/inputs", i, 1, true);
  }
  publishRefine();
  if (camera_ && calib_msg_.is_object()) {
    emit("calibration", calib_msg_, 1, true);
    publishMap();
  }
  if (cfg_.publish_snapshot) snapshot_sent_ = false;
}

void Pipeline::publishRefine() {
  auto h = msg::header("calibration_refine", cfg_.site_id, cfg_.camera_id, last_stamp_);
  nlohmann::json r = {{"status", refine_status_},
                      {"progress", refiner_.active() ? refiner_.progress() : 0.0},
                      {"tracks", refiner_.active() ? refiner_.usableTracks() : 0},
                      {"spread_deg", refine_spread_}};
  if (proposal_) {
    const auto& p = *proposal_;
    r["proposal"] = {{"spread_before_deg", p.spread_before_deg}, {"spread_after_deg", p.spread_after_deg},
                     {"points_before_m", p.points_before_m},     {"points_after_m", p.points_after_m},
                     {"d_pitch_deg", p.d_pitch_deg},             {"d_roll_deg", p.d_roll_deg},
                     {"d_yaw_deg", p.d_yaw_deg},                 {"d_focal_rel", p.d_focal_rel},
                     {"tracks", p.tracks},                       {"camera", p.camera.toJson()}};
  }
  h["refine"] = r;
  emit("calibration/refine", h, 1, true);
}

void Pipeline::renderPreview(const cv::Mat& frame, const std::vector<ImageDetection>& dets) {
  if (!preview_ || !preview_->wanted()) return;
  const double now = wallNow();
  if (t_preview_ >= 0 && now - t_preview_ < 1.0 / cfg_.preview_fps) return;
  t_preview_ = now;
  cv::Mat img;
  const double s = std::min(1.0, static_cast<double>(cfg_.preview_width) / frame.cols);
  cv::resize(frame, img, cv::Size(), s, s, cv::INTER_AREA);
  if (cfg_.preview_overlay) {
    // Colours match the panel (BGR).
    static const cv::Scalar colours[] = {{255, 141, 76}, {59, 212, 255}, {67, 159, 255}, {128, 160, 192},
                                         {102, 207, 81}, {252, 151, 177}, {230, 200, 60}, {149, 101, 240}};
    const int lw = std::max(1, static_cast<int>(std::round(img.cols / 640.0)));
    const double fs = 0.4 * img.cols / 960.0 + 0.1;
    if (camera_) {
      static const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
      for (const auto& u : scene_->users()) {
        if (u.state != TrackState::Confirmed) continue;
        const auto c = frame_cam_.projectBox({u.pos.x(), u.pos.y(), 0}, u.yaw, u.dims);
        if (!c) continue;
        const cv::Scalar col = colours[static_cast<size_t>(u.cls) % 8];
        std::array<cv::Point, 8> p;
        for (int k = 0; k < 8; ++k) p[k] = cv::Point(static_cast<int>((*c)[k].x() * s), static_cast<int>((*c)[k].y() * s));
        for (const auto& e : edges) cv::line(img, p[e[0]], p[e[1]], col, lw, cv::LINE_AA);
        int top = img.rows;
        int left = img.cols;
        for (const auto& q : p) top = std::min(top, q.y), left = std::min(left, q.x);
        char label[64];
        std::snprintf(label, sizeof(label), "%u %.0f km/h", u.id, std::abs(u.speed) * 3.6);
        int base = 0;
        const cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, fs, 1, &base);
        const cv::Point o(std::max(0, left), std::max(ts.height + 2, top - 3));
        cv::rectangle(img, o + cv::Point(-1, -ts.height - 2), o + cv::Point(ts.width + 2, base), col, cv::FILLED);
        cv::putText(img, label, o, cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
      }
    } else {
      for (const auto& d : dets) {
        if (d.score < tracker_.config().new_track_score) continue;
        const cv::Scalar col = colours[static_cast<size_t>(d.cls) % 8];
        cv::rectangle(img, cv::Point(static_cast<int>(d.box2d[0] * s), static_cast<int>(d.box2d[1] * s)),
                      cv::Point(static_cast<int>(d.box2d[2] * s), static_cast<int>(d.box2d[3] * s)), col, lw, cv::LINE_AA);
      }
    }
  }
  std::vector<uchar> buf;
  cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, 72});
  preview_->push(std::move(buf));
}

void Pipeline::process(const cv::Mat& frame, double stamp, double capture_wall) {
  last_stamp_ = stamp;
  std::deque<nlohmann::json> cmds;
  {
    std::lock_guard<std::mutex> lk(cmd_m_);
    cmds.swap(cmds_);
  }
  for (const auto& c : cmds) {
    try {
      handleCommand(c);
    } catch (const std::exception& e) {
      spdlog::warn("Invalid operator command: {}", e.what());
    }
  }
  cv::Mat net;
  lb_->apply(frame, net);
  auto dets = det_->detect(net, *lb_);
  dets.erase(std::remove_if(dets.begin(), dets.end(),
                            [this](const ImageDetection& d) { return !class_enabled_[static_cast<size_t>(d.cls)]; }),
             dets.end());
  if (src_->loops() != loops_seen_) {
    // A looping file jumped back to its start: old tracks would teleport.
    loops_seen_ = src_->loops();
    tracker_.reset();
    spdlog::info("Video restarted (loop {}): tracks reset", loops_seen_);
  }

  if (!camera_) {
    // The calibration view is also needed before any calibration exists (manual calibration).
    republishRetained(stamp);
    if (cfg_.publish_snapshot && !snapshot_sent_ && frames_ > 30) publishSnapshot(frame, dets);
    if (!cfg_.auto_calibration) {
      // Manual mode: wait for the operator's calibration (web panel).
    } else if (!calib_future_.valid()) {
      calibrator_->add(stamp, dets);
      if (calibrator_->ready()) {
        spdlog::info("Solving the auto-calibration ({} s of traffic)", cfg_.calibration_min_s);
        calib_future_ = std::async(std::launch::async, [c = calibrator_.get()] { return c->solve(); });
      }
    } else if (calib_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto res = calib_future_.get();
      if (res.ok) {
        camera_ = res.camera;
        onCalibrated(&res, "auto");
      } else {
        spdlog::warn("Calibration attempt failed: {}", res.message);
        calibrator_ = makeCalibrator();
      }
    }
    if (t_health_ < 0 || stamp - t_health_ >= 1.0 / cfg_.health_hz) {
      t_health_ = stamp;
      auto h = msg::header("health", cfg_.site_id, cfg_.camera_id, stamp);
      h["health"] = {{"level", "degraded"},
                     {"confidence", 0.0},
                     {"calibrated", false},
                     {"reasons", {cfg_.auto_calibration
                                      ? "auto-calibration in progress " + std::to_string(int(calibrator_->progress() * 100)) + "%"
                                      : std::string("waiting for manual calibration (Calibration tab)")}},
                     {"pipeline", {{"fps", fps_}, {"detector_ms", det_->lastLatencyMs()}}},
                     {"compute", computeInfo()},
                     {"source", {{"live", !src_->isFile()}, {"loop", src_->isFile() && cfg_.source.loop}}}};
      emit("health", h);
    }
    renderPreview(frame, dets);
    return;
  }

  // 1. Camera stability -> per-frame corrected camera.
  std::vector<Eigen::Vector4d> boxes;
  for (const auto& d : dets) boxes.push_back(d.box2d);
  const auto& stab = stability_.update(stamp, frame, cv::Matx33d(camera_->K()(0, 0), 0, camera_->K()(0, 2), 0,
                                                                 camera_->K()(1, 1), camera_->K()(1, 2), 0, 0, 1),
                                       boxes);
  frame_cam_ = (stab.status == "lost") ? frame_cam_ : camera_->rotated(stab.R_cur_ref);
  // A persistent, compensable rotation is absorbed into the calibration.
  if (stab.status == "drifted") {
    if (t_drift_ < 0) t_drift_ = stamp;
    if (stamp - t_drift_ > 30.0) {
      spdlog::info("Absorbing {:.2f} deg camera rotation into the calibration", stab.drift_deg);
      camera_ = camera_->rotated(stab.R_cur_ref);
      onCalibrated(nullptr, "drift-compensated");
      t_drift_ = -1;
      return;
    }
  } else {
    t_drift_ = -1;
  }

  // 2. Lift to 3D.
  std::vector<Detection3D> d3;
  for (const auto& d : dets) {
    auto x = frame_cam_.lift(d);
    if (!x || x->range >= cfg_.max_range) continue;
    if (cfg_.roi_filter && map_.roi.size() >= 3 && !insidePolygon(map_.roi, x->bottom.head<2>())) continue;
    d3.push_back(*x);
  }
  // 3. Track.
  auto occ_fn = [this](const Eigen::Vector2d& p) { return occlusion_.occluded(p); };
  const auto& tracks = tracker_.update(stamp, d3, occ_fn);
  // Paths of moving vehicles for the traffic check of the calibration.
  if (refine_status_ == "collecting") {
    for (const auto& t : tracks)
      if (t.state == TrackState::Confirmed && isVehicle(t.cls) && t.last_update == stamp && t.speed() > 2.0)
        refiner_.add(t.id, stamp, t.last_kp);
    if (refiner_.ready() && !refine_future_.valid()) {
      refine_status_ = "solving";
      refine_future_ = std::async(std::launch::async, [r = refiner_] { return r.solve(); });
      publishRefine();
    } else if (t_refine_pub_ < 0 || stamp - t_refine_pub_ > 5.0) {
      t_refine_pub_ = stamp;
      publishRefine();
    }
  } else if (refine_future_.valid() && refine_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    proposal_ = refine_future_.get();
    refine_spread_ = refiner_.currentSpread();
    refine_status_ = proposal_ ? "proposal" : "consistent";
    if (proposal_)
      spdlog::info("Calibration check: correction proposed (path spread {:.2f} -> {:.2f} deg)", proposal_->spread_before_deg,
                   proposal_->spread_after_deg);
    else
      spdlog::info("Calibration check: consistent with the traffic (path spread {:.2f} deg)", refine_spread_);
    publishRefine();
  }

  // 4. Lane learning from moving vehicles.
  for (const auto& t : tracks) {
    if (t.state != TrackState::Confirmed || !isVehicle(t.cls) || stamp - t.first_seen < 1.0) continue;
    learner_.addMovingSample(t.position(), t.yaw(), t.speed());
  }
  if ((t_lanes_ < 0 && learner_.samples() > 600) || (t_lanes_ >= 0 && stamp - t_lanes_ > 30.0)) {
    t_lanes_ = stamp;
    rebuildLanes(stamp);
  }

  // 5. Scene, safety.
  scene_->update(stamp, tracks, occ_fn);
  for (const auto& u : scene_->users()) {
    if (!u.queued || u.lane.empty() || !u.dist_to_stop || u.speed > 0.3) continue;
    bool leader = true;
    for (const auto& v : scene_->users())
      if (v.id != u.id && v.lane == u.lane && v.lane_s > u.lane_s && v.lane_s - u.lane_s < 12.0) leader = false;
    if (leader) learner_.addStoppedFront(u.lane, u.lane_s + u.dims[0] / 2);
  }
  const auto events = safety_->update(stamp, scene_->users());

  // 6. Self-monitoring.
  calmon_.update(stamp, frame_cam_, d3, scene_->users(), map_);
  if (seg_) {
    // Segmentation runs off the frame loop (slow on CPU); its result is applied when ready.
    if (seg_future_.valid() && seg_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      occlusion_.updateStatic(seg_future_.get(), frame_cam_);
    } else if (!seg_future_.valid() && (t_seg_ < 0 || stamp - t_seg_ > 2.0)) {
      t_seg_ = stamp;
      seg_future_ = std::async(std::launch::async, [this, f = frame.clone()] { return seg_->segment(f); });
    }
  }
  if (t_dyn_ < 0 || stamp - t_dyn_ > 0.2) {
    t_dyn_ = stamp;
    occlusion_.updateDynamic(frame_cam_, tracks);
  }
  if (t_vis_ < 0 || stamp - t_vis_ > 0.5) {
    t_vis_ = stamp;
    visibility_.update(stamp, frame, tracks);
  }
  // Self-healing: a calibration that stays invalid (or a camera that moved) triggers recalibration.
  // Manual calibrations are kept: the health topic reports the problem and the operator recalibrates.
  const bool bad = cfg_.auto_calibration && (calmon_.state().status == "invalid" || stab.status == "moved");
  if (bad) {
    if (t_invalid_ < 0) t_invalid_ = stamp;
    if (stamp - t_invalid_ > 60.0) {
      spdlog::warn("Calibration no longer valid ({}); starting auto-calibration", stab.status);
      camera_.reset();
      calibrator_ = makeCalibrator();
      t_invalid_ = -1;
      return;
    }
  } else {
    t_invalid_ = -1;
  }

  republishRetained(stamp);
  // 7. Outputs.
  renderPreview(frame, dets);
  latency_ms_ = (wallNow() - capture_wall) * 1000.0;
  if (cfg_.publish_snapshot && !snapshot_sent_) publishSnapshot(frame, dets);
  if (t_objects_ < 0 || stamp - t_objects_ >= 1.0 / cfg_.objects_hz - 1e-3) {
    t_objects_ = stamp;
    auto h = msg::header("objects", cfg_.site_id, cfg_.camera_id, stamp);
    h["frame"] = frames_;
    h["objects"] = msg::roadUsers(scene_->users(), map_);
    emit("objects", h);
  }
  if (t_cpm_ < 0 || stamp - t_cpm_ >= 1.0 / cfg_.cpm_hz - 1e-3) {
    t_cpm_ = stamp;
    emit("cpm", msg::cpm(scene_->users(), frame_cam_, cfg_.station_id, stamp,
                         std::max(visibility_.state().effective_range, 50.0)));
  }
  bool zone_change = prev_zone_occ_.size() != scene_->zones().size();
  for (size_t i = 0; !zone_change && i < scene_->zones().size(); ++i)
    zone_change = prev_zone_occ_[i] != scene_->zones()[i].occupied || scene_->zones()[i].entered_now > 0;
  if (zone_change || t_state_ < 0 || stamp - t_state_ >= 1.0 / cfg_.state_hz) {
    t_state_ = stamp;
    prev_zone_occ_.clear();
    for (const auto& z : scene_->zones()) prev_zone_occ_.push_back(z.occupied);
    auto h = msg::header("detectors", cfg_.site_id, cfg_.camera_id, stamp);
    h["zones"] = msg::zones(scene_->zones());
    h["lanes"] = msg::lanes(scene_->lanes());
    emit("detectors", h, 1, true);
    // One retained topic per virtual detector: the simplest controller interface (0/1 presence).
    for (const auto& z : scene_->zones())
      sink_->publish(topic("detector/" + z.id), z.occupied ? "1" : "0", 1, true);
  }
  for (const auto& c : scene_->crossings()) {
    auto h = msg::header("stopline_crossing", cfg_.site_id, cfg_.camera_id, stamp);
    h["crossing"] = msg::stopLineCrossing(c);
    emit("events/stopline", h, 1);
  }
  // Near-miss events are only as good as the geometry: suppressed under a low-confidence calibration.
  const bool safety_ok = calib_confidence_ >= cfg_.min_safety_confidence;
  for (const auto& e : safety_ok ? events : std::vector<NearMissEvent>{}) {
    auto h = msg::header("near_miss", cfg_.site_id, cfg_.camera_id, stamp);
    h["event"] = msg::nearMiss(e);
    emit("events/safety", h, 1);
    if (!e.finished && e.severity == "critical")
      spdlog::warn("Near-miss {} {}: {} #{} vs #{}", e.type, e.severity, e.event_id, e.a, e.b);
  }
  if (t_health_ < 0 || stamp - t_health_ >= 1.0 / cfg_.health_hz) {
    t_health_ = stamp;
    const auto hr = assessIntegrity(stab, visibility_.state(), calmon_.state(), occlusion_, fps_, cfg_.target_fps,
                                    latency_ms_, det_->lastLatencyMs(), true);
    auto h = msg::header("health", cfg_.site_id, cfg_.camera_id, stamp);
    h["health"] = msg::health(hr);
    h["health"]["pipeline"]["frames"] = frames_;
    h["health"]["pipeline"]["dropped"] = dropped_;
    h["health"]["calibration_confidence"] = calib_confidence_;
    h["health"]["compute"] = computeInfo();
    h["health"]["source"] = {{"live", !src_->isFile()}, {"loop", src_->isFile() && cfg_.source.loop}};
    h["health"]["preview_clients"] = preview_ ? preview_->clients() : 0;
    if (!safety_ok)
      h["health"]["reasons"].push_back("near-miss events suppressed: calibration confidence " +
                                         std::to_string(int(calib_confidence_ * 100)) + " %");
    emit("health", h, 1, true);
  }
  if (t_occ_ < 0 || stamp - t_occ_ >= 1.0 / cfg_.occlusion_hz) {
    t_occ_ = stamp;
    auto h = msg::header("occlusion", cfg_.site_id, cfg_.camera_id, stamp);
    h["occlusion"] = msg::occlusion(occlusion_);
    emit("occlusion", h);
  }
}

void Pipeline::run(std::uint64_t max_frames) {
  // Capture thread keeps only the newest frame; processing never works on stale video.
  std::mutex m;
  std::condition_variable cv;
  cv::Mat latest;
  double latest_stamp = 0, latest_wall = 0;
  bool have = false, eof = false;
  // Files without real-time pacing must not drop frames: capture waits for processing (lock-step).
  const bool lockstep = src_->isFile() && !cfg_.source.realtime;
  std::condition_variable cv_free;
  std::thread cap([&] {
    cv::Mat f;
    double t;
    while (!stop_) {
      if (!src_->read(f, t)) {
        std::lock_guard<std::mutex> lk(m);
        eof = true;
        cv.notify_one();
        return;
      }
      std::unique_lock<std::mutex> lk(m);
      if (lockstep) cv_free.wait(lk, [&] { return !have || stop_.load(); });
      if (have) ++dropped_;
      latest = f.clone();
      latest_stamp = t;
      latest_wall = wallNow();
      have = true;
      cv.notify_one();
    }
  });
  auto t_prev = std::chrono::steady_clock::now();
  while (!stop_) {
    cv::Mat f;
    double t, w;
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [&] { return have || eof || stop_.load(); });
      if (!have && eof) break;
      if (!have) continue;
      f = std::move(latest);
      t = latest_stamp;
      w = latest_wall;
      have = false;
      cv_free.notify_one();
    }
    process(f, t, w);
    ++frames_;
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - t_prev).count();
    t_prev = now;
    if (dt > 0) fps_ = fps_ == 0 ? 1.0 / dt : 0.95 * fps_ + 0.05 / dt;
    if (frames_ % 300 == 0)
      spdlog::info("frames={} fps={:.1f} latency={:.0f}ms detector={:.1f}ms tracks={} dropped={}", frames_, fps_,
                   latency_ms_, det_->lastLatencyMs(), tracker_.tracks().size(), dropped_);
    if (max_frames && frames_ >= max_frames) break;
  }
  stop_ = true;
  cv_free.notify_all();
  cap.join();
}

}  // namespace jp
