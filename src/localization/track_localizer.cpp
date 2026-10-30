#include "jp/localization/track_localizer.hpp"

#include <cmath>
#include <utility>

#include "jp/localization/ground_projection.hpp"

namespace jp::localization {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;

}  // namespace

std::optional<ContactModel> parse_contact_model(std::string_view text) {
  if (text == "none") {
    return ContactModel::kNone;
  }
  if (text == "ray_offset") {
    return ContactModel::kRayOffset;
  }
  if (text == "box_fit") {
    return ContactModel::kBoxFit;
  }
  return std::nullopt;
}

TrackLocalizer::TrackLocalizer(Eigen::Matrix3d image_to_ground, Eigen::Vector2d camera_position,
                               std::optional<CameraModel> camera, core::ClassCatalog catalog,
                               LocalizerOptions options)
    : image_to_ground_(std::move(image_to_ground)),
      camera_position_(std::move(camera_position)),
      camera_(std::move(camera)),
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
    const double sigma = options_.measurement_std_m +
                         (options_.measurement_std_per_m * (*contact - camera_position_).norm());

    auto it = states_.find(track.id);
    const std::optional<double> held_heading =
        it == states_.end() ? std::nullopt : it->second.heading_rad;
    const Eigen::Vector2d measured =
        footprint_center(*contact, track.detection.box, info, held_heading);

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

Eigen::Vector2d TrackLocalizer::footprint_center(const Eigen::Vector2d& contact,
                                                 const cv::Rect2f& box, const core::ClassInfo& info,
                                                 std::optional<double> heading_rad) const {
  if (options_.contact_model == ContactModel::kNone) {
    return contact;
  }
  const Eigen::Vector2d offset = correct_contact_bias(
      contact, camera_position_, Footprint{info.length_m, info.width_m}, heading_rad);
  if (options_.contact_model == ContactModel::kRayOffset || !camera_) {
    return offset;
  }
  const auto fitted = fit_footprint_center(
      *camera_, box, ObjectSize{info.length_m, info.width_m, info.height_m}, heading_rad, offset);
  return fitted.value_or(offset);
}

}  // namespace jp::localization
