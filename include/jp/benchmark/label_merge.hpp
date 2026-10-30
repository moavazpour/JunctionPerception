#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "jp/benchmark/label_registration.hpp"
#include "jp/benchmark/openlabel.hpp"

namespace jp::benchmark {

/// Which label set(s) a merged object was found in.
enum class LabelSource : std::uint8_t { kSouth, kNorth, kBoth };

/// Ground-truth object in the south LiDAR frame.
struct GroundTruthObject {
  std::int64_t track_id = 0;  ///< Stable over the sequence; shared by matched south/north tracks.
  Cuboid box;                 ///< For merged objects: the box with more LiDAR points.
  LabelSource source = LabelSource::kSouth;
};

struct MergeOptions {
  double gate_m = 2.0;              ///< BEV distance under which two boxes are one object.
  std::size_t min_pair_frames = 3;  ///< Frames a south/north track pair must match to be
                                    ///< treated as the same track.
};

struct MergeStatistics {
  std::size_t south_boxes = 0;
  std::size_t north_boxes = 0;
  std::size_t merged_boxes = 0;
  std::size_t matched_boxes = 0;  ///< Boxes present in both label sets.
  std::size_t tracks = 0;
};

struct MergedSequence {
  std::vector<std::vector<GroundTruthObject>> frames;
  MergeStatistics statistics;
};

/// Merges both label sets frame by frame. North boxes are moved into the south frame with
/// `south_from_north`; boxes of the same category closer than the gate are one object, of which
/// the box with more LiDAR points is kept unchanged (no averaging). Unmatched boxes of either set
/// are kept. Track IDs of matched south/north tracks are unified over the whole sequence.
[[nodiscard]] MergedSequence merge_label_sets(
    const std::vector<std::pair<std::vector<Cuboid>, std::vector<Cuboid>>>& frames,
    const PlanarPose& south_from_north, const MergeOptions& options = {});

}  // namespace jp::benchmark
