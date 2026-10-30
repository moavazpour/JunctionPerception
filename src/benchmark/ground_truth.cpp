#include "jp/benchmark/ground_truth.hpp"

#include <map>
#include <string>

namespace jp::benchmark {

namespace {

GroundTruth south_only(const TumtrafSequence& sequence) {
  GroundTruth truth;
  truth.labels = GroundTruthLabels::kSouth;
  std::map<std::string, std::int64_t, std::less<>> track_ids;
  truth.frames.reserve(sequence.size());
  for (const auto& frame : sequence.frames()) {
    std::vector<GroundTruthObject> objects;
    objects.reserve(frame.south_objects.size());
    for (const auto& box : frame.south_objects) {
      const auto [it, inserted] =
          track_ids.try_emplace(box.id, static_cast<std::int64_t>(track_ids.size()) + 1);
      objects.push_back({it->second, box, LabelSource::kSouth});
    }
    truth.boxes += objects.size();
    truth.frames.push_back(std::move(objects));
  }
  truth.tracks = track_ids.size();
  return truth;
}

GroundTruth merged(const TumtrafSequence& sequence, const RegistrationOptions& registration_options,
                   const MergeOptions& merge_options) {
  std::vector<std::pair<std::vector<Cuboid>, std::vector<Cuboid>>> pairs;
  pairs.reserve(sequence.size());
  for (const auto& frame : sequence.frames()) {
    pairs.emplace_back(frame.south_objects, frame.north_objects);
  }
  GroundTruth truth;
  truth.labels = GroundTruthLabels::kMerged;
  truth.registration = register_label_sets(pairs, registration_options);
  MergedSequence merged =
      merge_label_sets(pairs, truth.registration->south_from_north, merge_options);
  truth.frames = std::move(merged.frames);
  truth.merge = merged.statistics;
  truth.boxes = merged.statistics.merged_boxes;
  truth.tracks = merged.statistics.tracks;
  return truth;
}

}  // namespace

std::optional<GroundTruthLabels> parse_ground_truth_labels(std::string_view text) {
  if (text == "south") {
    return GroundTruthLabels::kSouth;
  }
  if (text == "merged") {
    return GroundTruthLabels::kMerged;
  }
  return std::nullopt;
}

GroundTruth load_ground_truth(const TumtrafSequence& sequence, GroundTruthLabels labels,
                              const RegistrationOptions& registration_options,
                              const MergeOptions& merge_options) {
  return labels == GroundTruthLabels::kSouth
             ? south_only(sequence)
             : merged(sequence, registration_options, merge_options);
}

}  // namespace jp::benchmark
