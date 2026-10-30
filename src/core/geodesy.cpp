#include "jp/core/geodesy.hpp"

#include <proj.h>

#include <cmath>
#include <format>
#include <stdexcept>
#include <string>

namespace jp::core {

// PROJ pipeline geographic (degrees) -> geocentric -> topocentric at the anchor. Built from a
// PROJ string only, so no PROJ database is needed at run time.
struct GeoAnchor::Projection {
  PJ_CONTEXT* context = nullptr;
  PJ* pipeline = nullptr;

  explicit Projection(const GeoPoint& origin) : context(proj_context_create()) {
    const std::string definition = std::format(
        "+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad "
        "+step +proj=cart +ellps=WGS84 "
        "+step +proj=topocentric +ellps=WGS84 +lat_0={:.12f} +lon_0={:.12f} +h_0={:.4f}",
        origin.lat_deg, origin.lon_deg, origin.alt_m);
    pipeline = proj_create(context, definition.c_str());
    if (pipeline == nullptr) {
      const std::string message = proj_context_errno_string(context, proj_context_errno(context));
      proj_context_destroy(context);
      throw std::runtime_error("PROJ: cannot create the ENU transformation: " + message);
    }
  }
  ~Projection() {
    proj_destroy(pipeline);
    proj_context_destroy(context);
  }
  Projection(const Projection&) = delete;
  Projection& operator=(const Projection&) = delete;
  Projection(Projection&&) = delete;
  Projection& operator=(Projection&&) = delete;

  [[nodiscard]] PJ_COORD apply(PJ_DIRECTION direction, PJ_COORD in) const {
    const PJ_COORD out = proj_trans(pipeline, direction, in);
    if (!std::isfinite(out.v[0]) || !std::isfinite(out.v[1])) {
      throw std::runtime_error("PROJ: coordinate transformation failed");
    }
    return out;
  }
};

GeoAnchor::GeoAnchor(GeoPoint origin)
    : origin_(origin), projection_(std::make_shared<const Projection>(origin)) {}

Eigen::Vector2d GeoAnchor::to_world(double lat_deg, double lon_deg) const {
  const PJ_COORD enu =
      projection_->apply(PJ_FWD, proj_coord(lon_deg, lat_deg, origin_.alt_m, 0.0));
  return {enu.enu.e, enu.enu.n};
}

GeoPoint GeoAnchor::to_geo(const Eigen::Vector2d& world) const {
  // Iterate the height so that the point lies at the anchor height, not on the tangent plane.
  PJ_COORD geo = projection_->apply(PJ_INV, proj_coord(world.x(), world.y(), 0.0, 0.0));
  for (int i = 0; i < 2; ++i) {
    const PJ_COORD enu =
        projection_->apply(PJ_FWD, proj_coord(geo.lp.lam, geo.lp.phi, origin_.alt_m, 0.0));
    geo = projection_->apply(PJ_INV, proj_coord(world.x(), world.y(), enu.enu.u, 0.0));
  }
  return {geo.lp.phi, geo.lp.lam, origin_.alt_m};
}

}  // namespace jp::core
