#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "jp/benchmark/openlabel.hpp"

namespace jp::benchmark {

inline constexpr std::string_view kSouthLidar = "s110_lidar_ouster_south";
inline constexpr std::string_view kNorthLidar = "s110_lidar_ouster_north";

/// A camera image of one sequence frame.
struct CameraImage {
  std::filesystem::path path;
  std::int64_t timestamp_ns = 0;
};

/// One synchronized frame: one image per camera and the two label sets.
/// South boxes are in the south LiDAR frame, north boxes in the north LiDAR frame.
struct SequenceFrame {
  std::size_t index = 0;
  std::map<std::string, CameraImage, std::less<>> images;
  std::vector<Cuboid> south_objects;
  std::vector<Cuboid> north_objects;
};

/// Reader for one TUMTraf Intersection R02 sequence folder (e.g. a9_dataset_r02_s03):
/// images/<camera>/*.jpg and labels_point_clouds/<lidar>/*.json.
class TumtrafSequence {
 public:
  /// Loads all label files of the sequence. Throws std::runtime_error if the folder layout or
  /// the pairing of south and north label files is inconsistent.
  [[nodiscard]] static TumtrafSequence open(const std::filesystem::path& sequence_dir);

  [[nodiscard]] const std::string& name() const { return name_; }
  [[nodiscard]] std::size_t size() const { return frames_.size(); }
  [[nodiscard]] const SequenceFrame& frame(std::size_t index) const { return frames_.at(index); }
  [[nodiscard]] const std::vector<SequenceFrame>& frames() const { return frames_; }

  /// Camera calibration relative to the south LiDAR (constant over the sequence).
  [[nodiscard]] const CameraCalibration& camera(std::string_view camera_name) const;
  [[nodiscard]] std::vector<std::string> camera_names() const;

 private:
  std::string name_;
  std::vector<SequenceFrame> frames_;
  std::map<std::string, CameraCalibration, std::less<>> cameras_;
};

}  // namespace jp::benchmark
