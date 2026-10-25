#include "jp/pipeline.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "jp/infer/detector.hpp"
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
  if (auto i = cam["intrinsics"]) {
    c.focal_px = opt<double>(i["focal_px"]);
    if (i["distortion"]) c.distortion = i["distortion"].as<std::vector<double>>();
  }
  if (auto k = n["calibration"]) {
    c.calibration_file = k["file"].as<std::string>("");
    c.force_recalibration = k["force"].as<bool>(false);
    c.road_bearing_deg = opt<double>(k["road_bearing_deg"]);
    c.calibration_min_s = k["min_duration_s"].as<double>(c.calibration_min_s);
    c.publish_snapshot = k["publish_snapshot"].as<bool>(false);
  }
  if (auto m = n["map"]) {
    c.map_file = m["file"].as<std::string>("");
    c.learn_lanes = m["learn_lanes"].as<bool>(true);
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
  det_ = std::make_unique<Detector3D>(dc);
  if (!cfg_.segmenter_onnx.empty()) seg_ = std::make_unique<Segmenter>(cfg_.segmenter_onnx, cfg_.fp16);
  sink_ = std::make_unique<MultiSink>();
  if (cfg_.mqtt) sink_->add(std::make_unique<MqttSink>(*cfg_.mqtt));
  if (!cfg_.metadata_file.empty()) sink_->add(std::make_unique<FileSink>(cfg_.metadata_file));

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
      onCalibrated(nullptr, "file");
    } else {
      spdlog::warn("Stored calibration is for a different resolution; recalibrating");
    }
  }
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
  if (res) {
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
  scene_->setMap(map_);
  occlusion_.configure(cam, map_, cfg_.max_range);
  nlohmann::json cj = msg::calibration(cam, res, source);
  cj["road_axis_yaw"] = axis;
  if (!cfg_.calibration_file.empty() && source != "file") {
    std::ofstream(cfg_.calibration_file) << cj.dump(1);
  }
  emit("calibration", [&] {
    auto h = msg::header("calibration", cfg_.site_id, cfg_.camera_id, last_stamp_);
    h["calibration"] = cj;
    return h;
  }(), 1, true);
  publishMap();
  snapshot_sent_ = false;
}

void Pipeline::publishMap() {
  if (!camera_) return;
  auto h = msg::header("map", cfg_.site_id, cfg_.camera_id, last_stamp_);
  h["map"] = map_.toJson();
  h["camera"] = camera_->toJson();
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
  h["jpeg_base64"] = base64(buf);
  h["note"] = "calibration view; road users pixelated";
  emit("snapshot", h, 1, true);
  snapshot_sent_ = true;
}

void Pipeline::rebuildLanes(double stamp) {
  if (map_from_file_ || !cfg_.learn_lanes || !camera_) return;
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

void Pipeline::process(const cv::Mat& frame, double stamp, double capture_wall) {
  last_stamp_ = stamp;
  cv::Mat net;
  lb_->apply(frame, net);
  const auto dets = det_->detect(net, *lb_);

  if (!camera_) {
    calibrator_->add(stamp, dets);
    if (calibrator_->ready()) {
      const auto res = calibrator_->solve();
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
                     {"reasons", {"auto-calibration in progress " + std::to_string(int(calibrator_->progress() * 100)) + "%"}},
                     {"pipeline", {{"fps", fps_}, {"detector_ms", det_->lastLatencyMs()}}}};
      emit("health", h);
    }
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
    if (x && x->range < cfg_.max_range) d3.push_back(*x);
  }
  // 3. Track.
  auto occ_fn = [this](const Eigen::Vector2d& p) { return occlusion_.occluded(p); };
  const auto& tracks = tracker_.update(stamp, d3, occ_fn);

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
  if (seg_ && (t_seg_ < 0 || stamp - t_seg_ > 2.0)) {
    t_seg_ = stamp;
    occlusion_.updateStatic(seg_->segment(frame), frame_cam_);
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
  const bool bad = calmon_.state().status == "invalid" || stab.status == "moved";
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

  // 7. Outputs.
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
  for (const auto& e : events) {
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
