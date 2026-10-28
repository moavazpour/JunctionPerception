#include "jp/scene/site_map.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <nlohmann/json.hpp>
#include <numeric>

namespace jp {
namespace {
nlohmann::json ptsToJson(const std::vector<Eigen::Vector2d>& pts) {
  nlohmann::json a = nlohmann::json::array();
  for (const auto& p : pts) a.push_back({std::round(p.x() * 100) / 100, std::round(p.y() * 100) / 100});
  return a;
}
std::vector<Eigen::Vector2d> ptsFromJson(const nlohmann::json& a) {
  std::vector<Eigen::Vector2d> out;
  for (const auto& p : a) out.emplace_back(p[0].get<double>(), p[1].get<double>());
  return out;
}
}  // namespace

const Lane* SiteMap::lane(const std::string& id) const {
  for (const auto& l : lanes)
    if (l.id == id) return &l;
  return nullptr;
}

nlohmann::json SiteMap::toJson() const {
  nlohmann::json j;
  j["name"] = name;
  j["lanes"] = nlohmann::json::array();
  for (const auto& l : lanes) {
    nlohmann::json lj = {{"id", l.id},           {"approach", l.approach}, {"to_junction", l.to_junction},
                         {"width", l.width},     {"source", l.source},     {"centerline", ptsToJson(l.centerline.pts)}};
    if (l.stop_s) lj["stop_s"] = *l.stop_s;
    j["lanes"].push_back(lj);
  }
  j["zones"] = nlohmann::json::array();
  for (const auto& z : zones)
    j["zones"].push_back({{"id", z.id}, {"type", z.type}, {"lane", z.lane}, {"polygon", ptsToJson(z.polygon)}});
  j["crosswalks"] = nlohmann::json::array();
  for (const auto& c : crosswalks) j["crosswalks"].push_back(ptsToJson(c));
  j["roi"] = ptsToJson(roi);
  return j;
}

SiteMap SiteMap::fromJson(const nlohmann::json& j) {
  SiteMap m;
  m.name = j.value("name", "");
  for (const auto& lj : j.value("lanes", nlohmann::json::array())) {
    Lane l;
    l.id = lj.at("id");
    l.approach = lj.value("approach", "");
    l.to_junction = lj.value("to_junction", true);
    l.width = lj.value("width", 3.25);
    l.source = lj.value("source", "config");
    l.centerline.pts = ptsFromJson(lj.at("centerline"));
    if (lj.contains("stop_s")) l.stop_s = lj["stop_s"].get<double>();
    m.lanes.push_back(l);
  }
  for (const auto& zj : j.value("zones", nlohmann::json::array())) {
    m.zones.push_back({zj.at("id"), zj.value("type", "presence"), zj.value("lane", ""), ptsFromJson(zj.at("polygon"))});
  }
  for (const auto& c : j.value("crosswalks", nlohmann::json::array())) m.crosswalks.push_back(ptsFromJson(c));
  if (j.contains("roi")) m.roi = ptsFromJson(j["roi"]);
  return m;
}

namespace {
// Polygon covering a stretch [s0, s1] of a lane.
std::vector<Eigen::Vector2d> laneStrip(const Lane& l, double s0, double s1) {
  std::vector<Eigen::Vector2d> left, right;
  const double L = l.centerline.length();
  s0 = std::clamp(s0, 0.0, L);
  s1 = std::clamp(s1, 0.0, L);
  const int n = std::max(2, static_cast<int>((s1 - s0) / 2.0) + 1);
  for (int i = 0; i < n; ++i) {
    const double s = s0 + (s1 - s0) * i / (n - 1);
    const Eigen::Vector2d p = l.centerline.at(s);
    const Eigen::Vector2d p2 = l.centerline.at(std::min(s + 0.5, L));
    const Eigen::Vector2d p1 = l.centerline.at(std::max(s - 0.5, 0.0));
    Eigen::Vector2d t = (p2 - p1).normalized();
    const Eigen::Vector2d nrm(-t.y(), t.x());
    left.push_back(p + nrm * l.width / 2);
    right.push_back(p - nrm * l.width / 2);
  }
  std::reverse(right.begin(), right.end());
  left.insert(left.end(), right.begin(), right.end());
  return left;
}
}  // namespace

void SiteMap::addDefaultDetectors(double stopbar_len, double advance_from, double advance_len) {
  for (const auto& l : lanes) {
    if (!l.to_junction || !l.stop_s) continue;
    const double s = *l.stop_s;
    const auto has = [&](const std::string& id) {
      return std::any_of(zones.begin(), zones.end(), [&](const Zone& z) { return z.id == id; });
    };
    if (!has(l.id + "_stopbar")) zones.push_back({l.id + "_stopbar", "presence", l.id, laneStrip(l, s - stopbar_len, s)});
    if (!has(l.id + "_advance"))
      zones.push_back({l.id + "_advance", "count", l.id, laneStrip(l, s - advance_from - advance_len, s - advance_from)});
  }
}

// ---------------------------------------------------------------- LaneLearner

void LaneLearner::addMovingSample(const Eigen::Vector2d& p, double yaw, double speed) {
  if (speed < cfg_.min_speed || p.norm() > cfg_.max_range) return;
  const Eigen::Vector2d ax(std::cos(axis_yaw_), std::sin(axis_yaw_));
  const double d = std::cos(yaw - axis_yaw_);
  if (std::abs(d) < 0.85) return;  // turning or crossing traffic
  const double s = p.dot(ax);
  const double t = ax.x() * p.y() - ax.y() * p.x();
  samples_.push_back({s, t, d > 0 ? 1 : -1});
  if (samples_.size() > 200000) samples_.erase(samples_.begin(), samples_.begin() + 50000);
}

void LaneLearner::addStoppedFront(const std::string& lane_id, double s_front) {
  stops_.emplace_back(lane_id, s_front);
  if (stops_.size() > 5000) stops_.erase(stops_.begin(), stops_.begin() + 1000);
}

std::optional<double> LaneLearner::learnedStopLine(const std::string& lane_id) const {
  std::vector<double> v;
  for (const auto& [id, s] : stops_)
    if (id == lane_id) v.push_back(s);
  if (v.size() < 3) return std::nullopt;
  std::sort(v.begin(), v.end());
  // The front-most queued vehicles define the stop line; use a high percentile to reject creeping.
  return v[static_cast<size_t>(0.9 * (v.size() - 1))] + 0.5;
}

std::vector<Lane> LaneLearner::build() const {
  std::vector<Lane> lanes;
  if (static_cast<int>(samples_.size()) < cfg_.min_samples) return lanes;
  const Eigen::Vector2d ax(std::cos(axis_yaw_), std::sin(axis_yaw_)), nrm(-ax.y(), ax.x());
  for (int dir : {1, -1}) {
    std::vector<const Sample*> ss;
    for (const auto& s : samples_)
      if (s.dir == dir) ss.push_back(&s);
    if (static_cast<int>(ss.size()) < cfg_.min_samples / 4) continue;
    // Lateral histogram, smoothed.
    double tmin = 1e9, tmax = -1e9;
    for (auto* s : ss) {
      tmin = std::min(tmin, s->t);
      tmax = std::max(tmax, s->t);
    }
    const int nb = static_cast<int>((tmax - tmin) / cfg_.bin) + 1;
    std::vector<double> h(nb, 0.0), hs(nb, 0.0);
    for (auto* s : ss) h[static_cast<int>((s->t - tmin) / cfg_.bin)] += 1;
    const int k = static_cast<int>(0.6 / cfg_.bin);
    for (int i = 0; i < nb; ++i)
      for (int j = std::max(0, i - k); j <= std::min(nb - 1, i + k); ++j) hs[i] += h[j];
    // Peaks separated by at least min_lane_sep with enough support.
    std::vector<int> idx(nb);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return hs[a] > hs[b]; });
    std::vector<double> centers;
    const double max_h = hs[idx[0]];
    for (int i : idx) {
      if (hs[i] < 0.12 * max_h) break;
      const double t = tmin + (i + 0.5) * cfg_.bin;
      if (std::all_of(centers.begin(), centers.end(), [&](double c) { return std::abs(c - t) >= cfg_.min_lane_sep; }))
        centers.push_back(t);
    }
    std::sort(centers.begin(), centers.end());
    int li = 0;
    for (double c : centers) {
      // Curved centreline: median lateral offset per longitudinal segment, for samples near this lane.
      std::map<int, std::vector<double>> segs;
      for (auto* s : ss)
        if (std::abs(s->t - c) < cfg_.min_lane_sep / 2) segs[static_cast<int>(std::floor(s->s / cfg_.seg_len))].push_back(s->t);
      std::vector<Eigen::Vector2d> pts;
      for (auto& [seg, ts] : segs) {
        if (ts.size() < 15) continue;
        std::nth_element(ts.begin(), ts.begin() + ts.size() / 2, ts.end());
        const double s = (seg + 0.5) * cfg_.seg_len;
        pts.push_back(ax * s + nrm * ts[ts.size() / 2]);
      }
      if (pts.size() < 2) continue;
      if (dir < 0) std::reverse(pts.begin(), pts.end());
      Lane l;
      const bool inbound = (dir > 0) == junction_ahead_;
      l.id = std::string(inbound ? "in" : "out") + std::to_string(++li) + (dir > 0 ? "f" : "b");
      l.to_junction = inbound;
      l.centerline.pts = pts;
      l.source = "learned";
      l.stop_s = learnedStopLine(l.id);
      lanes.push_back(l);
    }
  }
  // Lane width from spacing of neighbouring centres.
  for (auto& l : lanes) l.width = 3.25;
  return lanes;
}

std::optional<LaneHint> laneHintAt(const std::vector<Lane>& lanes, const Eigen::Vector2d& p, double yaw,
                                   double end_margin) {
  // Candidates: lanes the point lies in (with a small margin) whose direction roughly matches the heading.
  // Opposite-direction lanes never count; lanes of different direction overlapping here make it ambiguous.
  constexpr double kMaxAngle = 60.0 * M_PI / 180, kAmbiguous = 15.0 * M_PI / 180, kMargin = 0.5;
  const Lane* best = nullptr;
  Polyline::Projection best_p;
  std::vector<double> headings;
  for (const auto& l : lanes) {
    if (l.centerline.pts.size() < 2) continue;
    const auto pr = l.centerline.project(p);
    if (std::abs(pr.lateral) > l.width / 2 + kMargin || pr.dist > l.width / 2 + kMargin) continue;
    if (std::abs(wrapAngle(yaw - pr.heading)) > kMaxAngle) continue;
    headings.push_back(pr.heading);
    if (!best || std::abs(pr.lateral) < std::abs(best_p.lateral)) best = &l, best_p = pr;
  }
  if (!best) return std::nullopt;
  for (double h : headings)
    if (std::abs(wrapAngle(h - best_p.heading)) > kAmbiguous) return std::nullopt;
  if (best_p.s < end_margin || best_p.s > best->centerline.length() - end_margin) return std::nullopt;
  return LaneHint{best_p.heading, best_p.lateral, best->width};
}

}  // namespace jp
