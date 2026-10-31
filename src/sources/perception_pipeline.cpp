#include "jp/sources/perception_pipeline.hpp"

#include <chrono>
#include <utility>

namespace jp::sources {

tracking::TrackerOptions tracker_options(const core::TrackingSettings& settings) {
  tracking::TrackerOptions o;
  o.high_score = settings.high_score;
  o.low_score = settings.low_score;
  o.new_track_score = settings.new_track_score;
  o.high_match_min_iou = settings.high_match_min_iou;
  o.low_match_min_iou = settings.low_match_min_iou;
  o.tentative_match_min_iou = settings.tentative_match_min_iou;
  o.confirm_frames = settings.confirm_frames;
  o.max_missed_frames = settings.max_missed_frames;
  o.min_rider_votes = settings.min_rider_votes;
  return o;
}

localization::LocalizerOptions localizer_options(const core::LocalizationSettings& settings) {
  localization::LocalizerOptions o;
  o.contact_model =
      localization::parse_contact_model(settings.bias_correction).value_or(o.contact_model);
  o.acceleration_std_mps2 = settings.acceleration_std_mps2;
  o.class_acceleration_std_mps2 = settings.class_acceleration_std_mps2;
  o.measurement_pixel_std = settings.measurement_pixel_std;
  o.measurement_floor_std_m = settings.measurement_floor_std_m;
  o.initial_speed_std_mps = settings.initial_speed_std_mps;
  o.innovation_gate = settings.innovation_gate;
  o.reanchor_after = settings.reanchor_after;
  o.heading_min_speed_mps = settings.heading_min_speed_mps;
  o.forget_after_s = settings.forget_after_s;
  return o;
}

PerceptionPipeline::PerceptionPipeline(core::PipelineConfig config, core::ClassCatalog catalog,
                                       std::optional<calibration::GroundCalibration> ground)
    : config_(std::move(config)),
      catalog_(std::move(catalog)),
      ground_(std::move(ground)),
      detector_(detection::make_onnx_detector(
          {config_.detection.model, config_.detection.confidence_threshold,
           config_.detection.use_cuda, config_.detection.cuda_device})),
      tracker_(tracker_options(config_.tracking)) {
  riders_.min_person_overlap = config_.riders.min_person_overlap;
  riders_.min_feet_depth = config_.riders.min_feet_depth;
  reset();
}

void PerceptionPipeline::reset() {
  tracker_ = tracking::ByteTracker(tracker_options(config_.tracking));
  localizer_.reset();
  if (ground_) {
    std::optional<localization::CameraModel> camera;
    if (ground_->ground_projection) {
      camera = localization::CameraModel{*ground_->ground_projection, ground_->width_px,
                                         ground_->height_px};
    }
    localizer_.emplace(ground_->image_to_ground, ground_->camera_ground_position_m, camera,
                       catalog_, localizer_options(config_.localization));
  }
}

PerceptionPipeline::Result PerceptionPipeline::process(const io::Frame& frame) {
  const auto start = std::chrono::steady_clock::now();
  Result result;
  result.detections = detection::map_to_twin_classes(detector_->detect(frame.image), catalog_,
                                                     config_.detection.class_source);
  if (config_.riders.enabled) {
    result.detections = detection::associate_riders(std::move(result.detections), riders_);
  }
  result.tracks = tracker_.update(result.detections);
  if (localizer_) {
    result.objects = localizer_->update(result.tracks, frame.timestamp_ns);
  }
  result.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  return result;
}

}  // namespace jp::sources
