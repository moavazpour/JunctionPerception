#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "jp/core/class_catalog.hpp"
#include "jp/localization/bev_kalman.hpp"
#include "jp/localization/box_fit.hpp"
#include "jp/localization/keypoint_footprint.hpp"
#include "jp/tracking/byte_tracker.hpp"

namespace jp::localization {

/// How the visible ground contact (box bottom centre) is turned into the footprint centre.
enum class ContactModel : std::uint8_t {
  kNone,       ///< Use the contact point as is.
  kRayOffset,  ///< Shift along the camera ray by the class footprint (closed-form approximation).
  kBoxFit,     ///< Fit a class-sized 3D box to the 2D detection with the camera model; falls
               ///< back to kRayOffset for boxes touching the image border or failed fits.
};

/// Parses "none", "ray_offset" or "box_fit".
[[nodiscard]] std::optional<ContactModel> parse_contact_model(std::string_view text);

struct LocalizerOptions {
  ContactModel contact_model = ContactModel::kBoxFit;
  /// Process noise (white acceleration) per twin class; classes not listed use the default.
  double acceleration_std_mps2 = 2.0;
  std::map<std::string, double, std::less<>> class_acceleration_std_mps2;
  /// Measurement noise: image noise of the box bottom (px) mapped through the homography, which
  /// makes it anisotropic (large along the camera ray, small across it) and range dependent,
  /// plus an isotropic floor for the footprint model and the calibration.
  double measurement_pixel_std = 2.0;
  double measurement_floor_std_m = 0.5;
  double initial_speed_std_mps = 10.0;  ///< Velocity std of a new (or re-anchored) track.
  /// Innovation gate: squared Mahalanobis distance (chi-square, 2 dof) above which a measurement
  /// is not fused; 0 disables gating. After `reanchor_after` rejected frames in a row the track
  /// is re-anchored at the measurement and its velocity is uncertain again (e.g. a swapped or
  /// merged box, or a jump of the footprint model).
  double innovation_gate = 18.4;
  int reanchor_after = 3;
  double heading_min_speed_mps = 0.5;  ///< Below this speed the last heading is held.
  /// Heading from motion (detections without corners): the speed must also exceed this many
  /// standard deviations of its estimate, so the jitter of a standing object turns nothing.
  double heading_speed_sigma = 2.0;
  /// Weight of a new motion direction in the smoothed heading (1: no smoothing).
  double heading_smoothing = 0.6;
  /// Largest turn of a vehicle's motion heading (deg/s; 0: unlimited). VRUs turn freely.
  double heading_max_turn_rate_dps = 90.0;
  /// Keypoint models: the footprint from the four bottom corners replaces the contact model, its
  /// front and rear corners give the heading (also of standing objects) and its sides the size.
  bool use_keypoints = true;
  KeypointFootprintOptions keypoints;
  /// Isotropic floor of a corner measurement (it needs no footprint model, only the calibration).
  double keypoint_floor_std_m = 0.6;
  /// Weight of a new corner measurement in the smoothed heading and size.
  double keypoint_smoothing = 0.3;
  /// Heading of moving road users from the corners too (else from their motion, the corners
  /// orient only slow and standing ones).
  bool keypoint_heading_when_moving = true;
  /// Above this speed a corner heading against the motion is taken as reversed by mistake.
  double keypoint_motion_min_speed_mps = 2.0;
  /// Below it, a reversed corner heading turns the object after this many frames in a row.
  int keypoint_flip_frames = 5;
  /// Position from the centre of the corners (else from the box with the contact model).
  bool keypoint_position = false;
  /// Weight of the corner centre against the box fit in that position (1: corners only).
  double keypoint_position_weight = 1.0;
  double forget_after_s = 5.0;  ///< Filters of tracks unseen this long are dropped.
};

/// A tracked object on the ground plane.
struct LocalizedObject {
  std::int64_t id = 0;
  std::string twin_class;
  bool rider = false;
  float confidence = 0.0F;
  cv::Rect2f box;                                      ///< Image box of this frame.
  Eigen::Vector2d contact = Eigen::Vector2d::Zero();   ///< Box bottom centre on the ground.
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  ///< Filtered footprint centre.
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double speed_mps = 0.0;
  double speed_std_mps = 0.0;  ///< Uncertainty of the speed (filter covariance).
  /// Empty until the object has moved fast enough (or its corners gave it).
  std::optional<double> heading_rad;
  /// Measured footprint size (keypoint models), smoothed over the track.
  std::optional<double> length_m;
  std::optional<double> width_m;
  bool from_keypoints = false;  ///< This frame's corner keypoints were used (heading, size).
  std::vector<detection::Keypoint> corners;  ///< Bottom corners of this frame (image px), if any.
};

/// Image tracks -> ground plane: box bottom centre through the image-to-ground homography,
/// shift from the visible contact to the footprint centre (see ContactModel), a
/// constant-velocity Kalman filter per track, and heading from velocity (held while slow).
/// Detections with four bottom-corner keypoints are placed at the centre of their projected
/// footprint instead, which also gives heading and size.
class TrackLocalizer {
 public:
  /// `camera` is required for ContactModel::kBoxFit; without it the ray offset is used.
  TrackLocalizer(Eigen::Matrix3d image_to_ground, Eigen::Vector2d camera_position,
                 std::optional<CameraModel> camera, core::ClassCatalog catalog,
                 LocalizerOptions options = {});

  [[nodiscard]] std::vector<LocalizedObject> update(
      const std::vector<tracking::TrackedObject>& tracks, std::int64_t timestamp_ns);

 private:
  [[nodiscard]] Eigen::Vector2d footprint_center(const Eigen::Vector2d& contact,
                                                 const cv::Rect2f& box, const core::ClassInfo& info,
                                                 std::optional<double> heading_rad) const;

  struct State {
    explicit State(BevKalman initial) : filter(std::move(initial)) {}

    BevKalman filter;
    std::optional<double> heading_rad;
    std::optional<Eigen::Vector2d> size;  ///< Smoothed length and width from the corners.
    int rejected = 0;                     ///< Consecutive measurements outside the innovation gate.
    int reversed = 0;                     ///< Consecutive corner headings against the held one.
    std::int64_t heading_ns = 0;          ///< Time of the last motion heading update.
  };

  /// Heading of a state from its motion: gated on the speed's certainty, smoothed and turned at
  /// a limited rate.
  void apply_motion(State& state, const Eigen::Vector2d& velocity, double speed_std, bool vru,
                    std::int64_t timestamp_ns) const;

  /// Covariance of a ground position measured at `pixel`.
  [[nodiscard]] Eigen::Matrix2d measurement_cov(const Eigen::Vector2d& pixel,
                                                double floor_std_m) const;
  /// Heading and size of a state from a corner footprint.
  void apply_footprint(State& state, const GroundFootprint& footprint, double speed,
                       const Eigen::Vector2d& velocity) const;
  [[nodiscard]] BevKalman new_filter(const Eigen::Vector2d& position, const Eigen::Matrix2d& cov,
                                     std::int64_t timestamp_ns,
                                     const std::string& twin_class) const;
  /// Predicts and fuses one measurement, with gating and re-anchoring.
  void fuse(State& state, const Eigen::Vector2d& measured, const Eigen::Matrix2d& cov,
            std::int64_t timestamp_ns) const;

  Eigen::Matrix3d image_to_ground_;
  Eigen::Vector2d camera_position_;
  std::optional<CameraModel> camera_;
  core::ClassCatalog catalog_;
  LocalizerOptions options_;
  std::map<std::int64_t, State> states_;
};

}  // namespace jp::localization
