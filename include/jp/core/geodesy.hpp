#pragma once

#include <Eigen/Core>
#include <memory>

namespace jp::core {

/// Position on the WGS84 ellipsoid.
struct GeoPoint {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;  ///< Ellipsoidal height.
};

/// Local east-north-up frame tangent to the WGS84 ellipsoid at a scene's geo anchor. The world
/// frame of a geo-referenced scene: x east, y north (metres), ground z = 0 at the anchor height.
class GeoAnchor {
 public:
  /// Throws std::runtime_error if PROJ cannot create the transformation.
  explicit GeoAnchor(GeoPoint origin);

  [[nodiscard]] const GeoPoint& origin() const { return origin_; }

  /// East/north of a geographic position (its height is taken as the anchor height).
  [[nodiscard]] Eigen::Vector2d to_world(double lat_deg, double lon_deg) const;
  /// Geographic position of an east/north point at the anchor height.
  [[nodiscard]] GeoPoint to_geo(const Eigen::Vector2d& world) const;

 private:
  struct Projection;
  GeoPoint origin_;
  std::shared_ptr<const Projection> projection_;
};

}  // namespace jp::core
