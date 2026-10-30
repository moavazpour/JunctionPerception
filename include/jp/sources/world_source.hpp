#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <string>
#include <vector>

#include "jp/calibration/ground_calibration.hpp"
#include "jp/core/class_catalog.hpp"
#include "jp/core/pipeline_config.hpp"
#include "jp/core/world_state.hpp"
#include "jp/localization/track_localizer.hpp"

namespace jp::sources {

/// One frame of a world source: objects in the world frame (zones, turning flags and events are
/// added downstream) and, if requested and available, the camera image.
struct SourceFrame {
  std::int64_t index = 0;
  std::int64_t timestamp_ns = 0;
  std::vector<core::WorldObject> objects;
  cv::Mat image;  ///< BGR; empty if not requested or not available.
};

/// Producer of world frames: "live" (camera pipeline), "oracle" (benchmark ground truth) or
/// "replay" (recorded JSONL). All feed the same zone, event and output code.
class WorldSource {
 public:
  WorldSource() = default;
  WorldSource(const WorldSource&) = delete;
  WorldSource& operator=(const WorldSource&) = delete;
  WorldSource(WorldSource&&) = delete;
  WorldSource& operator=(WorldSource&&) = delete;
  virtual ~WorldSource() = default;

  [[nodiscard]] virtual std::optional<SourceFrame> next(bool with_image) = 0;
  /// Continues at `frame` (clamped to the source); false if the source cannot seek.
  virtual bool seek(std::int64_t frame) = 0;
  [[nodiscard]] virtual std::optional<std::int64_t> frame_count() const = 0;
  /// "live", "oracle" or "replay".
  [[nodiscard]] virtual std::string kind() const = 0;
};

/// World object of a localized track (class properties from the catalog).
[[nodiscard]] core::WorldObject to_world_object(const localization::LocalizedObject& object,
                                                const core::ClassCatalog& catalog);

/// Camera video through the perception pipeline.
[[nodiscard]] std::unique_ptr<WorldSource> make_live_source(
    const std::filesystem::path& video, const core::PipelineConfig& config,
    const core::ClassCatalog& catalog, const calibration::GroundCalibration& ground);

/// TUMTraf south LiDAR labels of a sequence, in the scene's ground frame; velocities from the
/// label positions over time, headings from the box orientation; images of the scene camera.
[[nodiscard]] std::unique_ptr<WorldSource> make_oracle_source(
    const std::filesystem::path& sequence_dir, const core::ClassCatalog& catalog,
    const calibration::GroundCalibration& ground);

/// Recorded world states (JSONL); zones and events are recomputed downstream.
[[nodiscard]] std::unique_ptr<WorldSource> make_replay_source(const std::filesystem::path& jsonl);

}  // namespace jp::sources
