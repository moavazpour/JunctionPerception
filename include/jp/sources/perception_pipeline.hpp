#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/core/pipeline_config.hpp"
#include "jp/detection/detector.hpp"
#include "jp/detection/rider_association.hpp"
#include "jp/io/frame_source.hpp"
#include "jp/localization/track_localizer.hpp"
#include "jp/tracking/byte_tracker.hpp"

namespace jp::sources {

/// Detection -> rider association -> tracking -> ground-plane localization for one frame.
class PerceptionPipeline {
 public:
  struct Result {
    std::vector<detection::Detection> detections;
    std::vector<tracking::TrackedObject> tracks;
    std::vector<localization::LocalizedObject> objects;  ///< Empty without a ground calibration.
    double elapsed_ms = 0.0;
  };

  /// Loads the detector; localization runs only with a ground calibration.
  PerceptionPipeline(core::PipelineConfig config, core::ClassCatalog catalog,
                     std::optional<calibration::GroundCalibration> ground);

  [[nodiscard]] Result process(const io::Frame& frame);
  /// Drops all tracks (e.g. after seeking); the detector is kept.
  void reset();

  [[nodiscard]] const detection::Detector& detector() const { return *detector_; }
  [[nodiscard]] bool localizes() const { return localizer_.has_value(); }

 private:
  core::PipelineConfig config_;
  core::ClassCatalog catalog_;
  std::optional<calibration::GroundCalibration> ground_;
  std::unique_ptr<detection::Detector> detector_;
  detection::RiderAssociationOptions riders_;
  tracking::ByteTracker tracker_;
  std::optional<localization::TrackLocalizer> localizer_;
};

[[nodiscard]] tracking::TrackerOptions tracker_options(const core::TrackingSettings& settings);
[[nodiscard]] localization::LocalizerOptions localizer_options(
    const core::LocalizationSettings& settings);

}  // namespace jp::sources
