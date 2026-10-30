// Scene description, map layers, calibration and zone editing endpoints of the twin server.

#include <spdlog/spdlog.h>

#include <fstream>
#include <stdexcept>

#include "jp/calibration/control_points.hpp"
#include "jp/vru/zones.hpp"
#include "server_impl.hpp"

namespace jp::server {

namespace {

using json = nlohmann::json;

json pair(const Eigen::Vector2d& v) { return json::array({v.x(), v.y()}); }

std::vector<calibration::ControlPoint> points_from(const json& body) {
  std::vector<calibration::ControlPoint> points;
  for (const auto& p : body.at("points")) {
    calibration::ControlPoint point;
    point.id = p.at("id").get<std::string>();
    point.pixel = {p.at("u").get<double>(), p.at("v").get<double>()};
    point.world = {p.at("x").get<double>(), p.at("y").get<double>()};
    point.check = p.value("check", false);
    points.push_back(std::move(point));
  }
  return points;
}

json points_json(const std::vector<calibration::ControlPoint>& points) {
  json list = json::array();
  for (const auto& p : points) {
    list.push_back({{"id", p.id},
                    {"u", p.pixel.x()},
                    {"v", p.pixel.y()},
                    {"x", p.world.x()},
                    {"y", p.world.y()},
                    {"check", p.check}});
  }
  return list;
}

json fit_json(const calibration::ControlPointFit& fit) {
  json residuals = json::array();
  for (const auto& r : fit.residuals) {
    residuals.push_back(
        {{"id", r.id}, {"check", r.check}, {"inlier", r.inlier}, {"error_m", r.error_m}});
  }
  json image_to_ground = json::array();
  for (Eigen::Index r = 0; r < 3; ++r) {
    image_to_ground.push_back(
        {fit.image_to_ground(r, 0), fit.image_to_ground(r, 1), fit.image_to_ground(r, 2)});
  }
  json result = {{"residuals", residuals},
                 {"image_to_ground", image_to_ground},
                 {"fit_points", fit.fit_points},
                 {"inliers", fit.inliers},
                 {"fit_rms_m", fit.fit_rms_m},
                 {"check_mean_m", fit.check_mean_m ? json(*fit.check_mean_m) : json(nullptr)},
                 {"check_max_m", fit.check_max_m ? json(*fit.check_max_m) : json(nullptr)},
                 {"camera_position_m", nullptr},
                 {"camera_height_m", nullptr}};
  if (fit.camera_position) {
    result["camera_position_m"] = pair(fit.camera_position->head<2>());
    result["camera_height_m"] = fit.camera_position->z();
  }
  return result;
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot write " + path.string());
  }
  out << text << '\n';
}

}  // namespace

std::string scene_description(const ServedScene& scene, const core::ClassCatalog& catalog,
                              bool has_map) {
  json classes = json::array();
  for (const auto& c : catalog.classes()) {
    classes.push_back({{"name", c.name},
                       {"length_m", c.length_m},
                       {"width_m", c.width_m},
                       {"height_m", c.height_m},
                       {"vru", c.vru}});
  }
  json description = {{"scene_id", scene.config.scene_id},
                      {"kind", scene.config.kind},
                      {"image_size_px", {scene.config.width_px, scene.config.height_px}},
                      {"calibrated", scene.ground.has_value()},
                      {"camera_ground_position_m", nullptr},
                      {"camera_height_m", nullptr},
                      {"geo_anchor", nullptr},
                      {"map", has_map ? json("/map/map.json") : json(nullptr)},
                      {"zones", json::parse(vru::zones_to_geojson(scene.zones))},
                      {"classes", classes}};
  if (scene.ground) {
    description["camera_ground_position_m"] = pair(scene.ground->camera_ground_position_m);
    description["camera_height_m"] = scene.ground->camera_height_m;
  }
  if (scene.config.geo_anchor) {
    const auto& a = *scene.config.geo_anchor;
    description["geo_anchor"] = {
        {"lat_deg", a.lat_deg}, {"lon_deg", a.lon_deg}, {"alt_m", a.alt_m}};
  }
  return description.dump();
}

void TwinServer::Impl::register_scene_routes() {
  CROW_ROUTE(app, "/api/scene")
  ([this] {
    const std::scoped_lock lock(scene_mutex);
    crow::response response(
        scene_description(scene, catalog, std::filesystem::exists(map_dir / "map.json")));
    response.set_header("Content-Type", "application/json");
    return response;
  });
  CROW_ROUTE(app, "/map/<path>")
  ([this](const crow::request& /*request*/, crow::response& response, const std::string& path) {
    serve_file(map_dir, path, response);
  });
  CROW_ROUTE(app, "/api/frame")
  ([this] {
    const std::scoped_lock lock(scene_mutex);
    if (latest_raw_jpeg.empty()) {
      return crow::response(404, "no camera image yet");
    }
    crow::response response(latest_raw_jpeg);
    response.set_header("Content-Type", "image/jpeg");
    return response;
  });
  CROW_ROUTE(app, "/api/calibration")
  ([this] {
    const std::scoped_lock lock(scene_mutex);
    const auto path = scene.config.control_points_path();
    if (!scene.config.geo_anchor || !std::filesystem::exists(path)) {
      return json_response({{"points", json::array()}});
    }
    const core::GeoAnchor anchor(*scene.config.geo_anchor);
    return json_response({{"points", points_json(calibration::load_control_points(path, anchor))}});
  });
  CROW_ROUTE(app, "/api/calibration/fit")
      .methods(crow::HTTPMethod::POST)([this](const crow::request& r) {
        try {
          const auto points = points_from(json::parse(r.body));
          return json_response(fit_json(calibration::fit_control_points(
              points, scene.config.width_px, scene.config.height_px)));
        } catch (const std::exception& error) {
          return json_response({{"error", error.what()}}, 400);
        }
      });
  CROW_ROUTE(app, "/api/calibration/save")
      .methods(crow::HTTPMethod::POST)([this](const crow::request& r) {
        try {
          const auto body = json::parse(r.body);
          const auto points = points_from(body);
          const std::scoped_lock lock(scene_mutex);
          if (!scene.config.geo_anchor) {
            throw std::runtime_error("the scene has no geo anchor");
          }
          const auto fit = calibration::fit_control_points(points, scene.config.width_px,
                                                           scene.config.height_px);
          std::optional<Eigen::Vector2d> camera;
          if (body.contains("camera_position_m") && !body["camera_position_m"].is_null()) {
            camera = Eigen::Vector2d(body["camera_position_m"][0].get<double>(),
                                     body["camera_position_m"][1].get<double>());
          }
          std::optional<double> height;
          if (body.contains("camera_height_m") && !body["camera_height_m"].is_null()) {
            height = body["camera_height_m"].get<double>();
          }
          const auto calibration =
              calibration::calibration_from_fit(fit, scene.config.scene_id, scene.config.width_px,
                                                scene.config.height_px, camera, height);
          calibration::write_control_points(scene.config.control_points_path(), points,
                                            core::GeoAnchor(*scene.config.geo_anchor));
          calibration::write_ground_calibration(scene.config.calibration_path(), calibration, fit);
          spdlog::info("saved calibration of {} ({} points, fit RMS {:.2f} m)",
                       scene.config.scene_id, points.size(), fit.fit_rms_m);
          return json_response(fit_json(fit));
        } catch (const std::exception& error) {
          return json_response({{"error", error.what()}}, 400);
        }
      });
  CROW_ROUTE(app, "/api/zones").methods(crow::HTTPMethod::POST)([this](const crow::request& r) {
    try {
      auto zones = vru::parse_zones(r.body);  // world frame, "coordinate_frame" declared
      const std::scoped_lock lock(scene_mutex);
      std::optional<core::GeoAnchor> anchor;
      if (scene.config.geo_anchor) {
        anchor.emplace(*scene.config.geo_anchor);
      }
      write_text(scene.config.zones_path(),
                 vru::zones_to_geojson(zones, anchor ? &*anchor : nullptr));
      scene.zones = zones;
      {
        const std::scoped_lock control(control_mutex);
        pending_zones = std::move(zones);
      }
      spdlog::info("saved {} zones of {}", scene.zones.size(), scene.config.scene_id);
      return json_response(json::parse(vru::zones_to_geojson(scene.zones)));
    } catch (const std::exception& error) {
      return json_response({{"error", error.what()}}, 400);
    }
  });
  CROW_ROUTE(app, "/api/restart").methods(crow::HTTPMethod::POST)([this] {
    spdlog::info("restart requested");
    app.stop();
    return crow::response(202);
  });
}

}  // namespace jp::server
