#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "jp/benchmark/label_merge.hpp"
#include "jp/benchmark/label_registration.hpp"
#include "jp/benchmark/tumtraf_sequence.hpp"

namespace jp::benchmark {

/// Which TUMTraf label sets form the ground truth.
enum class GroundTruthLabels : std::uint8_t {
  kSouth,   ///< South LiDAR labels only.
  kMerged,  ///< South and north labels, registered and merged.
};

/// Parses "south" or "merged".
[[nodiscard]] std::optional<GroundTruthLabels> parse_ground_truth_labels(std::string_view text);

/// Ground truth of a sequence, every frame in the south LiDAR frame.
struct GroundTruth {
  GroundTruthLabels labels = GroundTruthLabels::kSouth;
  std::vector<std::vector<GroundTruthObject>> frames;
  std::size_t boxes = 0;
  std::size_t tracks = 0;
  std::optional<RegistrationResult> registration;  ///< Set for merged labels only.
  std::optional<MergeStatistics> merge;            ///< Set for merged labels only.
};

[[nodiscard]] GroundTruth load_ground_truth(const TumtrafSequence& sequence,
                                            GroundTruthLabels labels,
                                            const RegistrationOptions& registration_options = {},
                                            const MergeOptions& merge_options = {});

}  // namespace jp::benchmark
