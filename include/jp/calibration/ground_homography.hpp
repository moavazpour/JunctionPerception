#pragma once

#include <Eigen/Geometry>
#include <optional>

#include "jp/calibration/ground_plane.hpp"

namespace jp::calibration {

/// Metric frame on a ground plane: z = 0 on the plane, z up, origin below `origin_hint`, x along
/// the sensor's x axis projected onto the plane.
[[nodiscard]] Eigen::Isometry3d sensor_from_ground_frame(const Plane& plane,
                                                         const Eigen::Vector3d& origin_hint);

/// Homography from ground-plane coordinates (x, y, 1) in metres to homogeneous image pixels,
/// for a camera with projection matrix `projection` (sensor frame to pixels).
[[nodiscard]] Eigen::Matrix3d ground_to_image_homography(
    const Eigen::Matrix<double, 3, 4>& projection, const Eigen::Isometry3d& sensor_from_ground);

/// Applies a homography; empty when the point maps to infinity.
[[nodiscard]] std::optional<Eigen::Vector2d> apply_homography(const Eigen::Matrix3d& h,
                                                              const Eigen::Vector2d& point);

/// Homography normalised to h(2, 2) = 1.
[[nodiscard]] Eigen::Matrix3d normalized_homography(const Eigen::Matrix3d& h);

/// Recovers a full camera projection (ground frame -> pixels) from a ground-to-image homography,
/// assuming square pixels and the principal point at the image centre: the focal length follows
/// from the orthogonality of the rotation columns, the third column from their cross product.
/// Empty if the homography is not consistent with such a camera.
[[nodiscard]] std::optional<Eigen::Matrix<double, 3, 4>> projection_from_homography(
    const Eigen::Matrix3d& ground_to_image, int width_px, int height_px);

/// Camera projection scaled to metres: unit principal axis (third row of the left 3x3) and the
/// sign that puts `seen_ground_point`, a ground point the camera sees, in front of it. Its third
/// projected coordinate is then the depth in metres, which box fit gates on; a projection built
/// from a homography has the homography's arbitrary scale until then.
[[nodiscard]] Eigen::Matrix<double, 3, 4> metric_projection(
    const Eigen::Matrix<double, 3, 4>& projection, const Eigen::Vector2d& seen_ground_point);

/// Ground point at the image centre, through an image-to-ground homography.
[[nodiscard]] Eigen::Vector2d ground_at_image_center(const Eigen::Matrix3d& image_to_ground,
                                                     int width_px, int height_px);

/// Camera projection (ground frame -> pixels) consistent with a ground-to-image homography and a
/// known camera centre (x, y, height): the third column follows from P * (centre, 1) = 0. Needs
/// no intrinsics, so it also works where the homography cannot constrain the focal length (long
/// lenses, control points along a single road). Empty if the centre is on the ground plane.
[[nodiscard]] std::optional<Eigen::Matrix<double, 3, 4>> projection_from_camera_center(
    const Eigen::Matrix3d& ground_to_image, const Eigen::Vector3d& center, int width_px,
    int height_px);

}  // namespace jp::calibration
