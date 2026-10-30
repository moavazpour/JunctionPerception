#include "jp/localization/track_localizer.hpp"

#include <cmath>
#include <utility>

#include "jp/localization/ground_projection.hpp"

namespace jp::localization {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;

}  // namespace

TrackLocalizer::TrackLocalizer(Eigen::Matrix3d image_to_ground, Eigen::Vector2d camera_position,
                               core::ClassCatalog catalog, LocalizerOptions options)
    : image_to_ground_(std::move(image_to_ground)),
      camera_position_(std::move(camera_position)),
      catalog_(std::move(catalog)),
      options_(options) {}

std::vector<LocalizedObject> TrackLocalizer::update(
    const std::vector<tracking::TrackedObject>& tracks, std::int64_t timestamp_ns) {
  std::vector<LocalizedObject> objects;
  objects.reserve(tracks.size());
  for (const auto& track : tracks) {
    const auto contact = ground_contact(track.detection.box, image_to_ground_);
    if (!contact) {
      continue;
    }
    const auto& info = catalog_.info(track.twin_class);
    const Footprint footprint{info.length_m, info.width_m};
    const double sigma = options_.measurement_std_m +
                         (options_.measurement_std_per_m * (*contact - camera_position_).norm());

    auto it = states_.find(track.id);
    const std::optional<double> held_heading =
        it == states_.end() ? std::nullopt : it->second.heading_rad;
    const Eigen::Vector2d measured =
        options_.bias_correction
            ? correct_contact_bias(*contact, camera_position_, footprint, held_heading)
            : *contact;

    if (it == states_.end()) {
      it = states_
               .emplace(track.id, State(BevKalman(measured, timestamp_ns, sigma,
                                                  options_.acceleration_std_mps2)))
               .first;
    } else {
      it->second.filter.predict(timestamp_ns);
      it->second.filter.update(measured, sigma);
    }
    State& state = it->second;
    const Eigen::Vector2d velocity = state.filter.velocity();
    const double speed = velocity.norm();
    if (speed >= options_.heading_min_speed_mps) {
      state.heading_rad = std::atan2(velocity.y(), velocity.x());
    }

    LocalizedObject object;
    object.id = track.id;
    object.twin_class = track.twin_class;
    object.rider = track.rider;
    object.confidence = track.detection.score;
    object.box = track.detection.box;
    object.contact = *contact;
    object.position = state.filter.position();
    object.velocity = velocity;
    object.speed_mps = speed;
    object.heading_rad = state.heading_rad;
    objects.push_back(std::move(object));
  }

  const auto forget_ns = static_cast<std::int64_t>(options_.forget_after_s * kNanosecondsPerSecond);
  std::erase_if(states_, [&](const auto& entry) {
    return timestamp_ns - entry.second.filter.timestamp_ns() > forget_ns;
  });
  return objects;
}

}  // namespace jp::localization
