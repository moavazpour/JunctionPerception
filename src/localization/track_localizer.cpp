#include "jp/localization/track_localizer.hpp"

#include <cmath>
#include <numbers>
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
      options_(std::move(options)) {}

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
    std::optional<GroundFootprint> footprint;
    if (options_.use_keypoints) {
      footprint = keypoint_footprint(track.detection.keypoints, image_to_ground_,
                                     ObjectSize{info.length_m, info.width_m, info.height_m},
                                     options_.keypoints);
    }

    auto it = states_.find(track.id);
    const std::optional<double> held_heading =
        it == states_.end() ? std::nullopt : it->second.heading_rad;
    // The corners can give the footprint centre directly; otherwise it is estimated from the box
    // (with the heading the corners keep for standing road users).
    const bool corner_position = footprint && options_.keypoint_position;
    const Eigen::Matrix2d cov =
        corner_position
            ? measurement_cov(footprint->pixel_center, options_.keypoint_floor_std_m)
            : measurement_cov(contact_pixel(track.detection.box), options_.measurement_floor_std_m);
    const Eigen::Vector2d measured =
        corner_position ? footprint->center
                        : footprint_center(*contact, track.detection.box, info, held_heading);

    if (it == states_.end()) {
      it = states_
               .emplace(track.id, State(new_filter(measured, cov, timestamp_ns, track.twin_class)))
               .first;
    } else {
      fuse(it->second, measured, cov, timestamp_ns);
    }
    State& state = it->second;
    const Eigen::Vector2d velocity = state.filter.velocity();
    const double speed = velocity.norm();
    if (footprint) {
      apply_footprint(state, *footprint, speed, velocity);
    } else if (speed >= options_.heading_min_speed_mps) {
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
    object.speed_std_mps = state.filter.speed_std();
    object.heading_rad = state.heading_rad;
    if (state.size) {
      object.length_m = state.size->x();
      object.width_m = state.size->y();
    }
    object.from_keypoints = footprint.has_value();
    object.corners = track.detection.keypoints;
    objects.push_back(std::move(object));
  }

  const auto forget_ns = static_cast<std::int64_t>(options_.forget_after_s * kNanosecondsPerSecond);
  std::erase_if(states_, [&](const auto& entry) {
    return timestamp_ns - entry.second.filter.timestamp_ns() > forget_ns;
  });
  return objects;
}

void TrackLocalizer::apply_footprint(State& state, const GroundFootprint& footprint, double speed,
                                     const Eigen::Vector2d& velocity) const {
  const double k = options_.keypoint_smoothing;
  const Eigen::Vector2d size(footprint.length_m, footprint.width_m);
  state.size = state.size ? Eigen::Vector2d(((1.0 - k) * *state.size) + (k * size)) : size;
  // The long side is the axis of the object; which end is the front follows its motion, else
  // the heading it had.
  std::optional<double> reference = state.heading_rad;
  if (speed >= options_.heading_min_speed_mps) {
    reference = std::atan2(velocity.y(), velocity.x());
    if (!options_.keypoint_heading_when_moving) {
      // The motion gives the heading of moving road users more steadily than the corners.
      state.heading_rad = reference;
      return;
    }
  }
  if (!reference) {
    // The corners do not tell the front from the back: without a motion seen so far the
    // heading stays unknown rather than half the time reversed.
    return;
  }
  const double axis = orient_axis(footprint.axis_rad, *reference);
  if (!state.heading_rad) {
    state.heading_rad = axis;
    return;
  }
  // Smoothed on the circle: turn the held heading part of the way towards the measurement.
  const double delta = std::remainder(axis - *state.heading_rad, 2.0 * std::numbers::pi);
  state.heading_rad = std::remainder(*state.heading_rad + (k * delta), 2.0 * std::numbers::pi);
}

Eigen::Matrix2d TrackLocalizer::measurement_cov(const Eigen::Vector2d& pixel,
                                                double floor_std_m) const {
  Eigen::Matrix2d cov = floor_std_m * floor_std_m * Eigen::Matrix2d::Identity();
  if (const auto jacobian = ground_jacobian(pixel, image_to_ground_)) {
    const double s = options_.measurement_pixel_std;
    cov += s * s * (*jacobian) * jacobian->transpose();
  }
  return cov;
}

BevKalman TrackLocalizer::new_filter(const Eigen::Vector2d& position, const Eigen::Matrix2d& cov,
                                     std::int64_t timestamp_ns,
                                     const std::string& twin_class) const {
  const auto accel = options_.class_acceleration_std_mps2.find(twin_class);
  return {position, timestamp_ns, cov,
          accel == options_.class_acceleration_std_mps2.end() ? options_.acceleration_std_mps2
                                                              : accel->second,
          options_.initial_speed_std_mps};
}

void TrackLocalizer::fuse(State& state, const Eigen::Vector2d& measured, const Eigen::Matrix2d& cov,
                          std::int64_t timestamp_ns) const {
  state.filter.predict(timestamp_ns);
  if (options_.innovation_gate > 0.0 &&
      state.filter.innovation_distance(measured, cov) > options_.innovation_gate) {
    // Not explained by the motion model: skip it, unless it persists (the object is elsewhere).
    if (++state.rejected < options_.reanchor_after) {
      return;
    }
    state.filter.reanchor(measured, cov, options_.initial_speed_std_mps);
    state.rejected = 0;
    return;
  }
  state.rejected = 0;
  state.filter.update(measured, cov);
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
