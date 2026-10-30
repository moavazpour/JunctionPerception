#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <utility>

#include "jp/benchmark/ground_truth.hpp"
#include "jp/benchmark/tumtraf_sequence.hpp"
#include "jp/core/rigid_transform.hpp"
#include "jp/io/frame_source.hpp"
#include "jp/sources/perception_pipeline.hpp"
#include "jp/sources/world_source.hpp"
#include "jp/twin/bev_plot.hpp"
#include "jp/twin/debug_overlay.hpp"
#include "jp/twin/world_state_json.hpp"

namespace jp::sources {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;

class LiveSource final : public WorldSource {
 public:
  LiveSource(std::filesystem::path video, const core::PipelineConfig& config,
             const core::ClassCatalog& catalog,
             const std::optional<calibration::GroundCalibration>& ground)
      : video_(std::move(video)),
        catalog_(catalog),
        pipeline_(config, catalog, ground),
        frames_(std::make_unique<io::VideoFileSource>(video_)) {}

  std::optional<SourceFrame> next(bool with_image) override {
    auto frame = frames_->next();
    if (!frame) {
      return std::nullopt;
    }
    const auto result = pipeline_.process(*frame);
    SourceFrame out;
    out.index = frame->index;
    out.timestamp_ns = frame->timestamp_ns;
    for (const auto& object : result.objects) {
      out.objects.push_back(to_world_object(object, catalog_));
    }
    if (with_image) {
      // The side panel shows the camera image with the tracked boxes and their IDs (only the
      // detections while the scene is not calibrated).
      out.image = pipeline_.localizes()
                      ? twin::draw_tracks(frame->image, result.objects, "")
                      : twin::draw_detections(frame->image, result.detections, "");
      out.raw_image = std::move(frame->image);
    }
    return out;
  }

  bool seek(std::int64_t frame) override {
    frames_ = std::make_unique<io::VideoFileSource>(video_);
    for (std::int64_t i = 0; i < frame; ++i) {
      if (!frames_->next()) {
        break;
      }
    }
    pipeline_.reset();
    return true;
  }

  [[nodiscard]] std::optional<std::int64_t> frame_count() const override {
    return frames_->frame_count();
  }
  [[nodiscard]] std::string kind() const override { return "live"; }

 private:
  std::filesystem::path video_;
  core::ClassCatalog catalog_;
  PerceptionPipeline pipeline_;
  std::unique_ptr<io::VideoFileSource> frames_;
};

class OracleSource final : public WorldSource {
 public:
  OracleSource(const std::filesystem::path& sequence_dir, const core::ClassCatalog& catalog,
               const calibration::GroundCalibration& ground) {
    if (!ground.sensor_from_ground) {
      throw std::runtime_error("oracle source needs a benchmark scene (sensor_from_ground)");
    }
    const auto sequence = benchmark::TumtrafSequence::open(sequence_dir);
    const auto truth = benchmark::load_ground_truth(sequence, benchmark::GroundTruthLabels::kSouth);
    const Eigen::Isometry3d ground_from_sensor = core::rigid_inverse(*ground.sensor_from_ground);

    for (std::size_t i = 0; i < sequence.size(); ++i) {
      const auto& image = sequence.frame(i).images.at(ground.camera);
      SourceFrame frame;
      frame.index = static_cast<std::int64_t>(i);
      frame.timestamp_ns = image.timestamp_ns;
      for (const auto& gt : truth.frames[i]) {
        const auto twin_class = catalog.map("tumtraf", gt.box.category);
        if (!twin_class) {
          continue;
        }
        const Eigen::Vector3d center = ground_from_sensor * gt.box.bottom_center();
        const Eigen::Vector3d forward =
            ground_from_sensor.linear() * (gt.box.rotation * Eigen::Vector3d::UnitX());
        core::WorldObject object;
        object.id = gt.track_id;
        object.object_class = *twin_class;
        object.vru = catalog.info(*twin_class).vru;
        object.x = center.x();
        object.y = center.y();
        object.heading_rad = std::atan2(forward.y(), forward.x());
        object.confidence = 1.0;
        frame.objects.push_back(std::move(object));
      }
      image_paths_.push_back(image.path);
      frames_.push_back(std::move(frame));
    }
    estimate_velocities();
  }

  std::optional<SourceFrame> next(bool with_image) override {
    if (position_ >= frames_.size()) {
      return std::nullopt;
    }
    SourceFrame frame = frames_[position_];
    if (with_image) {
      frame.image = cv::imread(image_paths_[position_].string(), cv::IMREAD_COLOR);
      frame.raw_image = frame.image;
    }
    ++position_;
    return frame;
  }

  bool seek(std::int64_t frame) override {
    position_ = static_cast<std::size_t>(
        std::clamp<std::int64_t>(frame, 0, static_cast<std::int64_t>(frames_.size())));
    return true;
  }

  [[nodiscard]] std::optional<std::int64_t> frame_count() const override {
    return static_cast<std::int64_t>(frames_.size());
  }
  [[nodiscard]] std::string kind() const override { return "oracle"; }

 private:
  // Central differences of each track's labelled positions over the frame timestamps.
  void estimate_velocities() {
    std::map<std::int64_t, std::vector<std::pair<std::size_t, std::size_t>>> by_track;
    for (std::size_t f = 0; f < frames_.size(); ++f) {
      for (std::size_t o = 0; o < frames_[f].objects.size(); ++o) {
        by_track[frames_[f].objects[o].id].emplace_back(f, o);
      }
    }
    for (const auto& [id, samples] : by_track) {
      for (std::size_t k = 0; k < samples.size(); ++k) {
        const auto& [f0, o0] = samples[k == 0 ? 0 : k - 1];
        const auto& [f1, o1] = samples[std::min(k + 1, samples.size() - 1)];
        const double dt = static_cast<double>(frames_[f1].timestamp_ns - frames_[f0].timestamp_ns) /
                          kNanosecondsPerSecond;
        auto& object = frames_[samples[k].first].objects[samples[k].second];
        if (dt > 0.0) {
          const auto& a = frames_[f0].objects[o0];
          const auto& b = frames_[f1].objects[o1];
          object.vx = (b.x - a.x) / dt;
          object.vy = (b.y - a.y) / dt;
          object.speed_mps = std::hypot(object.vx, object.vy);
        }
      }
    }
  }

  std::vector<SourceFrame> frames_;
  std::vector<std::filesystem::path> image_paths_;
  std::size_t position_ = 0;
};

class ReplaySource final : public WorldSource {
 public:
  explicit ReplaySource(const std::filesystem::path& jsonl) {
    twin::JsonlReader reader(jsonl);
    while (auto state = reader.next()) {
      SourceFrame frame;
      frame.index = state->frame;
      frame.timestamp_ns = state->timestamp_ns;
      frame.objects = std::move(state->objects);
      for (auto& object : frame.objects) {
        object.zones.clear();
        object.turning = false;
      }
      frames_.push_back(std::move(frame));
    }
  }

  std::optional<SourceFrame> next(bool /*with_image*/) override {
    if (position_ >= frames_.size()) {
      return std::nullopt;
    }
    return frames_[position_++];
  }

  bool seek(std::int64_t frame) override {
    const auto it = std::ranges::lower_bound(frames_, frame, {}, &SourceFrame::index);
    position_ = static_cast<std::size_t>(it - frames_.begin());
    return true;
  }

  [[nodiscard]] std::optional<std::int64_t> frame_count() const override {
    return static_cast<std::int64_t>(frames_.size());
  }
  [[nodiscard]] std::string kind() const override { return "replay"; }

 private:
  std::vector<SourceFrame> frames_;
  std::size_t position_ = 0;
};

}  // namespace

core::WorldObject to_world_object(const localization::LocalizedObject& object,
                                  const core::ClassCatalog& catalog) {
  core::WorldObject out;
  out.id = object.id;
  out.object_class = object.twin_class;
  out.vru = catalog.info(object.twin_class).vru;
  out.rider = object.rider;
  out.x = object.position.x();
  out.y = object.position.y();
  out.vx = object.velocity.x();
  out.vy = object.velocity.y();
  out.speed_mps = object.speed_mps;
  out.heading_rad = object.heading_rad;
  out.confidence = object.confidence;
  return out;
}

std::unique_ptr<WorldSource> make_live_source(const std::filesystem::path& video,
                                              const core::PipelineConfig& config,
                                              const core::ClassCatalog& catalog,
                                              const std::optional<calibration::GroundCalibration>& ground) {
  return std::make_unique<LiveSource>(video, config, catalog, ground);
}

std::unique_ptr<WorldSource> make_oracle_source(const std::filesystem::path& sequence_dir,
                                                const core::ClassCatalog& catalog,
                                                const calibration::GroundCalibration& ground) {
  return std::make_unique<OracleSource>(sequence_dir, catalog, ground);
}

std::unique_ptr<WorldSource> make_replay_source(const std::filesystem::path& jsonl) {
  return std::make_unique<ReplaySource>(jsonl);
}

}  // namespace jp::sources
