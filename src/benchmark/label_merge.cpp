#include "jp/benchmark/label_merge.hpp"

#include <algorithm>
#include <map>
#include <string>

namespace jp::benchmark {

namespace {

// Union-find over label-set track keys ("S:<uuid>" / "N:<uuid>").
class TrackUnion {
 public:
  std::size_t node(const std::string& key) {
    const auto [it, inserted] = index_.try_emplace(key, parent_.size());
    if (inserted) {
      parent_.push_back(parent_.size());
    }
    return it->second;
  }

  std::size_t root(std::size_t n) {
    while (parent_[n] != n) {
      parent_[n] = parent_[parent_[n]];
      n = parent_[n];
    }
    return n;
  }

  void unite(std::size_t a, std::size_t b) {
    a = root(a);
    b = root(b);
    if (a != b) {
      parent_[std::max(a, b)] = std::min(a, b);
    }
  }

 private:
  std::map<std::string, std::size_t> index_;
  std::vector<std::size_t> parent_;
};

std::string south_key(const Cuboid& box) { return "S:" + box.id; }
std::string north_key(const Cuboid& box) { return "N:" + box.id; }

using FramePairs = std::vector<std::pair<std::vector<Cuboid>, std::vector<Cuboid>>>;

struct FrameMatches {
  std::vector<Cuboid> north_moved;  // north boxes in the south frame
  std::vector<std::pair<std::size_t, std::size_t>> matches;
};

std::vector<FrameMatches> match_frames(const FramePairs& frames, const PlanarPose& pose,
                                       double gate_m) {
  std::vector<FrameMatches> per_frame;
  per_frame.reserve(frames.size());
  for (const auto& [south, north] : frames) {
    FrameMatches fm;
    fm.north_moved.reserve(north.size());
    for (const auto& box : north) {
      fm.north_moved.push_back(pose.apply(box));
    }
    fm.matches = match_boxes(south, fm.north_moved, gate_m);
    per_frame.push_back(std::move(fm));
  }
  return per_frame;
}

// Sequence-wide track IDs: south and north tracks matched in enough frames share one ID.
class TrackIds {
 public:
  TrackIds(const FramePairs& frames, const std::vector<FrameMatches>& per_frame,
           std::size_t min_pair_frames) {
    std::map<std::pair<std::string, std::string>, std::size_t> pair_counts;
    for (std::size_t f = 0; f < frames.size(); ++f) {
      const auto& [south, north] = frames[f];
      for (const auto& box : south) {
        tracks_.node(south_key(box));
      }
      for (const auto& box : north) {
        tracks_.node(north_key(box));
      }
      for (const auto& [si, ni] : per_frame[f].matches) {
        ++pair_counts[{south_key(south[si]), north_key(north[ni])}];
      }
    }
    for (const auto& [keys, count] : pair_counts) {
      if (count >= min_pair_frames) {
        tracks_.unite(tracks_.node(keys.first), tracks_.node(keys.second));
      }
    }
  }

  std::int64_t of(const std::string& key) {
    const std::size_t root = tracks_.root(tracks_.node(key));
    const auto [it, inserted] = ids_.try_emplace(root, static_cast<std::int64_t>(ids_.size()) + 1);
    return it->second;
  }

  [[nodiscard]] std::size_t count() const { return ids_.size(); }

 private:
  TrackUnion tracks_;
  std::map<std::size_t, std::int64_t> ids_;
};

std::vector<GroundTruthObject> merge_frame(const std::vector<Cuboid>& south, const FrameMatches& fm,
                                           TrackIds& track_ids) {
  std::vector<bool> south_used(south.size(), false);
  std::vector<bool> north_used(fm.north_moved.size(), false);
  std::vector<GroundTruthObject> objects;
  for (const auto& [si, ni] : fm.matches) {
    south_used[si] = true;
    north_used[ni] = true;
    const Cuboid& s = south[si];
    const Cuboid& n = fm.north_moved[ni];
    const bool keep_north = n.num_points > s.num_points;
    objects.push_back({keep_north ? track_ids.of(north_key(n)) : track_ids.of(south_key(s)),
                       keep_north ? n : s, LabelSource::kBoth});
  }
  for (std::size_t i = 0; i < south.size(); ++i) {
    if (!south_used[i]) {
      objects.push_back({track_ids.of(south_key(south[i])), south[i], LabelSource::kSouth});
    }
  }
  for (std::size_t j = 0; j < fm.north_moved.size(); ++j) {
    if (!north_used[j]) {
      objects.push_back(
          {track_ids.of(north_key(fm.north_moved[j])), fm.north_moved[j], LabelSource::kNorth});
    }
  }
  return objects;
}

}  // namespace

MergedSequence merge_label_sets(const FramePairs& frames, const PlanarPose& south_from_north,
                                const MergeOptions& options) {
  const auto per_frame = match_frames(frames, south_from_north, options.gate_m);
  TrackIds track_ids(frames, per_frame, options.min_pair_frames);

  MergedSequence result;
  result.frames.reserve(frames.size());
  for (std::size_t f = 0; f < frames.size(); ++f) {
    auto objects = merge_frame(frames[f].first, per_frame[f], track_ids);
    result.statistics.south_boxes += frames[f].first.size();
    result.statistics.north_boxes += frames[f].second.size();
    result.statistics.matched_boxes += per_frame[f].matches.size();
    result.statistics.merged_boxes += objects.size();
    result.frames.push_back(std::move(objects));
  }
  result.statistics.tracks = track_ids.count();
  return result;
}

}  // namespace jp::benchmark
