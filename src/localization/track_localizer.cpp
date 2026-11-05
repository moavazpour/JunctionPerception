#include "jp/localization/track_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "jp/localization/ground_projection.hpp"

namespace jp::localization {

namespace {

constexpr double kNanosecondsPerSecond = 1e9;

/// Whether another track's box covers the bottom edge of track `index`'s box: it reaches that
/// edge, ends lower in the image (nearer to a camera looking down at the road) and overlaps at
/// least `min_overlap` of the width. The contact point of such a box is not the road user's.
bool bottom_occluded(const std::vector<tracking::TrackedObject>& tracks, std::size_t index,
                     double min_overlap) {
  const cv::Rect2f& box = tracks[index].detection.box;
  const float bottom = box.y + box.height;
  for (std::size_t other = 0; other < tracks.size(); ++other) {
    const cv::Rect2f& nearer = tracks[other].detection.box;
    if (other == index || nearer.y + nearer.height <= bottom || nearer.y >= bottom) {
      continue;
    }
    const float overlap =
        std::min(box.x + box.width, nearer.x + nearer.width) - std::max(box.x, nearer.x);
    if (overlap >= static_cast<float>(min_overlap) * box.width) {
      return true;
    }
  }
  return false;
}

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
  for (std::size_t index = 0; index < tracks.size(); ++index) {
    const auto& track = tracks[index];
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
    Eigen::Matrix2d cov =
        corner_position
            ? measurement_cov(footprint->pixel_center, options_.keypoint_floor_std_m)
            : measurement_cov(contact_pixel(track.detection.box), options_.measurement_floor_std_m);
    if (bottom_occluded(tracks, index, options_.occlusion_min_overlap)) {
      cov *= options_.occlusion_noise_scale * options_.occlusion_noise_scale;
    }
    const Eigen::Vector2d box_center =
        footprint_center(*contact, track.detection.box, info, held_heading);
    // The corner centre lies slightly towards the camera; weighted with the box fit it keeps
    // the corners' robustness with less of that offset.
    const double w = options_.keypoint_position_weight;
    const Eigen::Vector2d measured =
        corner_position ? Eigen::Vector2d((w * footprint->center) + ((1.0 - w) * box_center))
                        : box_center;

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
    } else {
      apply_motion(state, velocity, state.filter.speed_std(), info.vru, timestamp_ns);
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
    object.heading_source = state.heading_rad ? state.heading_source : std::nullopt;
    object.position_cov = state.filter.position_cov();
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
  const double motion = std::atan2(velocity.y(), velocity.x());
  if (speed >= options_.heading_min_speed_mps && !options_.keypoint_heading_when_moving) {
    state.heading_rad = motion;
    state.heading_source = "motion";
    return;
  }
  state.heading_source = "keypoints";
  // The corners give the heading with its front. A reversal against a clear motion is a
  // momentary error of the model; against the held heading of a slow road user it counts once
  // it lasts (it turned, or the first frames were wrong).
  double heading = footprint.heading_rad;
  const bool moving = speed >= options_.keypoint_motion_min_speed_mps;
  const std::optional<double> reference = moving ? std::optional(motion) : state.heading_rad;
  if (reference && std::abs(std::remainder(heading - *reference, 2.0 * std::numbers::pi)) >
                       std::numbers::pi / 2.0) {
    if (moving || ++state.reversed < options_.keypoint_flip_frames) {
      heading = orient_axis(heading, *reference);
    } else {
      state.heading_rad = heading;
      state.reversed = 0;
      return;
    }
  } else {
    state.reversed = 0;
  }
  if (!state.heading_rad) {
    state.heading_rad = heading;
    return;
  }
  // Smoothed on the circle: turn the held heading part of the way towards the measurement.
  const double delta = std::remainder(heading - *state.heading_rad, 2.0 * std::numbers::pi);
  state.heading_rad = std::remainder(*state.heading_rad + (k * delta), 2.0 * std::numbers::pi);
}

void TrackLocalizer::apply_motion(State& state, const Eigen::Vector2d& velocity, double speed_std,
                                  bool vru, std::int64_t timestamp_ns) const {
  const double speed = velocity.norm();
  const Eigen::Vector2d position = state.filter.position();
  const bool slow =
      speed < options_.heading_min_speed_mps || speed < options_.heading_speed_sigma * speed_std;
  if (slow || !state.travel_origin) {
    state.travel_origin = position;
  }
  const bool travelled =
      vru || (position - *state.travel_origin).norm() >= options_.heading_min_travel_m;
  if (slow || !travelled) {
    // Too slow, or not far enough yet, to measure: the last heading, if any, is held.
    if (state.heading_rad) {
      state.heading_source = "held";
    }
    return;
  }
  const double motion = std::atan2(velocity.y(), velocity.x());
  state.heading_source = "motion";
  const std::int64_t last_ns = std::exchange(state.heading_ns, timestamp_ns);
  if (!state.heading_rad) {
    state.heading_rad = motion;
    return;
  }
  // Smoothed on the circle, then limited to what a vehicle can turn since the last update.
  double delta = options_.heading_smoothing *
                 std::remainder(motion - *state.heading_rad, 2.0 * std::numbers::pi);
  if (!vru && options_.heading_max_turn_rate_dps > 0.0) {
    const double dt = static_cast<double>(timestamp_ns - last_ns) / kNanosecondsPerSecond;
    const double limit = options_.heading_max_turn_rate_dps * std::numbers::pi / 180.0 * dt;
    delta = std::clamp(delta, -limit, limit);
  }
  state.heading_rad = std::remainder(*state.heading_rad + delta, 2.0 * std::numbers::pi);
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
