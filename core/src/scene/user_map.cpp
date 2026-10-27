#include "jp/scene/user_map.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include <optional>

namespace jp {
namespace {
nlohmann::json pts(const std::vector<Eigen::Vector2d>& v) {
  auto a = nlohmann::json::array();
  for (const auto& p : v) a.push_back({p.x(), p.y()});
  return a;
}
std::vector<Eigen::Vector2d> pts(const nlohmann::json& a) {
  std::vector<Eigen::Vector2d> v;
  for (const auto& p : a) v.emplace_back(p.at(0).get<double>(), p.at(1).get<double>());
  return v;
}

// Intersection of segment a-b with segment c-d: parameter t along a-b, or none.
std::optional<double> intersect(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c,
                                const Eigen::Vector2d& d) {
  const Eigen::Vector2d r = b - a, s = d - c;
  const double den = r.x() * s.y() - r.y() * s.x();
  if (std::abs(den) < 1e-12) return std::nullopt;
  const Eigen::Vector2d q = c - a;
  const double t = (q.x() * s.y() - q.y() * s.x()) / den, u = (q.x() * r.y() - q.y() * r.x()) / den;
  if (t < 0 || t > 1 || u < 0 || u > 1) return std::nullopt;
  return t;
}

std::string compass(double yaw) {
  // yaw: ENU heading (rad, from east, counter-clockwise) -> the compass sector it points to.
  static const char* names[] = {"east", "north", "west", "south"};
  const int k = static_cast<int>(std::lround(yaw / (M_PI / 2))) & 3;
  return names[k];
}
}  // namespace

bool insidePolygon(const std::vector<Eigen::Vector2d>& poly, const Eigen::Vector2d& p) {
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto &a = poly[i], &b = poly[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) && p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x()) in = !in;
  }
  return in;
}

nlohmann::json UserMap::toJson() const {
  nlohmann::json j;
  j["lanes"] = nlohmann::json::array();
  for (const auto& l : lanes) j["lanes"].push_back({{"id", l.id}, {"name", l.name}, {"points", pts(l.pts)}, {"width", l.width}});
  j["stop_lines"] = nlohmann::json::array();
  for (const auto& s : stop_lines) j["stop_lines"].push_back(pts(std::vector<Eigen::Vector2d>{s[0], s[1]}));
  j["crosswalks"] = nlohmann::json::array();
  for (const auto& c : crosswalks) j["crosswalks"].push_back(pts(c));
  j["roi"] = pts(roi);
  return j;
}

UserMap UserMap::fromJson(const nlohmann::json& j) {
  UserMap m;
  int k = 1;
  for (const auto& l : j.value("lanes", nlohmann::json::array())) {
    LaneGeo g;
    g.id = l.value("id", "");
    if (g.id.empty()) g.id = "L" + std::to_string(k);
    g.name = l.value("name", "");
    g.pts = pts(l.at("points"));
    g.width = std::clamp(l.value("width", 3.25), 1.0, 12.0);
    ++k;
    if (g.pts.size() >= 2) m.lanes.push_back(g);
  }
  for (const auto& s : j.value("stop_lines", nlohmann::json::array())) {
    const auto v = pts(s);
    if (v.size() >= 2) m.stop_lines.push_back({v.front(), v.back()});
  }
  for (const auto& c : j.value("crosswalks", nlohmann::json::array())) {
    auto v = pts(c);
    if (v.size() >= 3) m.crosswalks.push_back(v);
  }
  if (j.contains("roi")) {
    m.roi = pts(j["roi"]);
    if (m.roi.size() < 3) m.roi.clear();
  }
  return m;
}

SiteMap UserMap::build(const GeoAnchor& a, const std::string& name) const {
  SiteMap m;
  m.name = name;
  auto enu = [&](const Eigen::Vector2d& g) { return geoToEnu(a, g.x(), g.y()); };
  std::vector<std::array<Eigen::Vector2d, 2>> stops;
  for (const auto& s : stop_lines) stops.push_back({enu(s[0]), enu(s[1])});
  for (const auto& g : lanes) {
    Lane l;
    l.id = g.id;
    l.width = g.width;
    l.source = "user";
    for (const auto& p : g.pts) l.centerline.pts.push_back(enu(p));
    auto& c = l.centerline.pts;
    // Stop line: the first one the centreline crosses (the last segment is extended a little, so a lane
    // drawn up to the line also counts).
    double acc = 0;
    for (size_t i = 1; i < c.size() && !l.stop_s; ++i) {
      Eigen::Vector2d b = c[i];
      const double L = (c[i] - c[i - 1]).norm();
      if (i + 1 == c.size() && L > 1e-6) b = c[i] + (c[i] - c[i - 1]) / L * 4.0;
      for (const auto& s : stops) {
        // Stop lines are widened by a metre on each side against clicking short.
        const Eigen::Vector2d d = (s[1] - s[0]).normalized();
        if (auto t = intersect(c[i - 1], b, s[0] - d, s[1] + d)) {
          l.stop_s = acc + *t * (b - c[i - 1]).norm();
          break;
        }
      }
      acc += L;
    }
    l.to_junction = l.stop_s.has_value();
    if (l.to_junction) {
      // Inbound lanes are named by where traffic comes from.
      const double s = std::min(*l.stop_s, l.centerline.length());
      Eigen::Vector2d dir = c.back() - c.front();
      double acc2 = 0;
      for (size_t i = 1; i < c.size(); ++i) {
        const double L = (c[i] - c[i - 1]).norm();
        if (acc2 + L >= s - 1e-6) {
          dir = c[i] - c[i - 1];
          break;
        }
        acc2 += L;
      }
      l.approach = compass(std::atan2(-dir.y(), -dir.x()));
      // Make sure the centreline reaches a few metres past the stop line.
      const double len = l.centerline.length();
      if (*l.stop_s > len - 2.0) {
        const Eigen::Vector2d u = (c.back() - c[c.size() - 2]).normalized();
        c.push_back(c.back() + u * (*l.stop_s - len + 3.0));
      }
    } else {
      const Eigen::Vector2d dir = c.back() - c.front();
      l.approach = "exit " + compass(std::atan2(dir.y(), dir.x()));
    }
    m.lanes.push_back(l);
  }
  for (const auto& cw : crosswalks) {
    std::vector<Eigen::Vector2d> p;
    for (const auto& g : cw) p.push_back(enu(g));
    m.crosswalks.push_back(p);
  }
  for (const auto& g : roi) m.roi.push_back(enu(g));
  m.addDefaultDetectors();
  return m;
}

}  // namespace jp
