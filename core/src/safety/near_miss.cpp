#include "jp/safety/near_miss.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include "jp/geometry/bev.hpp"
#include "jp/tracking/tracker.hpp"

namespace jp {
namespace {

struct Pose {
  Eigen::Vector2d p;
  double yaw;
};

Pose predictCv(const RoadUser& u, double t) {
  return {u.pos + u.speed * t * Eigen::Vector2d(std::cos(u.yaw), std::sin(u.yaw)), u.yaw};
}

Pose predictCtra(const RoadUser& u, double t) {
  CtraEkf::Vec s;
  // A decelerating object stops; it does not reverse.
  double a = u.accel, tt = t;
  if (a < 0 && u.speed + a * t < 0) tt = u.speed / -a;
  s << u.pos.x(), u.pos.y(), u.yaw, u.speed, std::clamp(u.yaw_rate, -0.8, 0.8), a;
  const auto x = CtraEkf::f(s, tt);
  return {x.head<2>(), x[2]};
}

// Pose on a lane at arc length s, keeping the current lateral offset.
Pose lanePose(const Polyline& c, double s, double lateral) {
  const Eigen::Vector2d d = c.at(s + 0.5) - c.at(s - 0.5);
  const double h = d.norm() > 1e-6 ? std::atan2(d.y(), d.x()) : c.project(c.at(s)).heading;
  return {c.at(s) + lateral * Eigen::Vector2d(-std::sin(h), std::cos(h)), h};
}

// Lane following: advances along the lane centreline with the user's speed (and acceleration); past the lane end it
// continues with CTRA (or constant velocity) from the end of the lane.
Pose predictLane(const RoadUser& u, const Lane& l, double t, bool accel) {
  const double a = accel ? u.accel : 0.0;
  const double len = l.centerline.length();
  const auto t_end = IntersectionModel::timeToDistance(len - u.lane_s, u.speed, a);
  if (!t_end || t <= *t_end)
    return lanePose(l.centerline, u.lane_s + IntersectionModel::distanceInTime(t, u.speed, a), u.lateral);
  const Pose end = lanePose(l.centerline, len, u.lateral);
  RoadUser e = u;
  e.pos = end.p;
  e.yaw = end.yaw;
  e.speed = std::max(0.0, u.speed + a * *t_end);
  e.accel = a;
  return accel ? predictCtra(e, t - *t_end) : predictCv(e, t - *t_end);
}

OrientedBox box(const RoadUser& u, const Pose& p, double shrink) {
  // Pedestrians and two-wheelers get a minimum footprint so contact is not missed.
  const double l = std::max(u.dims[0], 0.6) * shrink, w = std::max(u.dims[1], 0.6) * shrink;
  return {p.p, p.yaw, l, w};
}

std::string conflictType(const RoadUser& a, const RoadUser& b) {
  if (!isVehicle(a.cls) || !isVehicle(b.cls)) return "vru_vehicle";
  const double d = std::abs(wrapAngle(a.yaw - b.yaw));
  if (d < M_PI / 6) return "rear_end";
  if (d > 5 * M_PI / 6) return "head_on";
  return "crossing";
}

std::int64_t cellKey(int x, int y) { return (static_cast<std::int64_t>(x) << 32) ^ static_cast<std::uint32_t>(y); }

}  // namespace

ConflictMetrics NearMissDetector::evaluatePair(const RoadUser& a, const RoadUser& b) const {
  ConflictMetrics m;
  const OrientedBox ba = box(a, {a.pos, a.yaw}, 1.0), bb = box(b, {b.pos, b.yaw}, 1.0);
  m.gap = distance(ba, bb);
  const Eigen::Vector2d va = a.speed * Eigen::Vector2d(std::cos(a.yaw), std::sin(a.yaw));
  const Eigen::Vector2d vb = b.speed * Eigen::Vector2d(std::cos(b.yaw), std::sin(b.yaw));
  const Eigen::Vector2d dp = b.pos - a.pos;
  const double dist = std::max(dp.norm(), 1e-6);
  m.closing_speed = -(vb - va).dot(dp) / dist;
  if (m.closing_speed <= 0.3) return m;
  // Road users on a lane follow it; the others keep their heading.
  const Lane* la = map_ && !a.lane.empty() ? map_->lane(a.lane) : nullptr;
  const Lane* lb = map_ && !b.lane.empty() ? map_->lane(b.lane) : nullptr;
  auto predict = [](const RoadUser& u, const Lane* l, double t, bool accel) {
    if (l && l->centerline.pts.size() >= 2) return predictLane(u, *l, t, accel);
    return accel ? predictCtra(u, t) : predictCv(u, t);
  };
  for (double t = 0; t <= cfg_.horizon + 1e-9; t += cfg_.step) {
    const Pose pa = predict(a, la, t, false), pb = predict(b, lb, t, false);
    if (overlaps(box(a, pa, cfg_.footprint_shrink), box(b, pb, cfg_.footprint_shrink))) {
      m.ttc = t;
      m.conflict_point = 0.5 * (pa.p + pb.p);
      break;
    }
  }
  for (double t = 0; t <= cfg_.horizon + 1e-9; t += cfg_.step) {
    const Pose pa = predict(a, la, t, true), pb = predict(b, lb, t, true);
    if (overlaps(box(a, pa, cfg_.footprint_shrink), box(b, pb, cfg_.footprint_shrink))) {
      m.mttc = t;
      if (!m.ttc) m.conflict_point = 0.5 * (pa.p + pb.p);
      break;
    }
  }
  // Deceleration needed to avoid a crash only means something on a collision course: cars passing each other on
  // parallel lanes close in fast but never meet.
  if ((m.ttc || m.mttc) && m.closing_speed > 0.5 && m.gap > 0.05)
    m.drac = m.closing_speed * m.closing_speed / (2 * m.gap);
  return m;
}

void NearMissDetector::updatePetGrid(
    double stamp, const std::vector<RoadUser>& users,
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<double, Eigen::Vector2d>>& pets) {
  const double c = cfg_.cell;
  for (const auto& u : users) {
    if (u.speed < 0.5) continue;  // waiting users do not encroach
    const OrientedBox fb = box(u, {u.pos, u.yaw}, 1.0);
    const auto cs = fb.corners();
    double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    for (const auto& p : cs) {
      x0 = std::min(x0, p.x());
      x1 = std::max(x1, p.x());
      y0 = std::min(y0, p.y());
      y1 = std::max(y1, p.y());
    }
    const std::vector<Eigen::Vector2d> poly(cs.begin(), cs.end());
    for (int ix = static_cast<int>(std::floor(x0 / c)); ix <= static_cast<int>(std::floor(x1 / c)); ++ix) {
      for (int iy = static_cast<int>(std::floor(y0 / c)); iy <= static_cast<int>(std::floor(y1 / c)); ++iy) {
        const Eigen::Vector2d centre((ix + 0.5) * c, (iy + 0.5) * c);
        if (!pointInPolygon(poly, centre)) continue;
        Cell& cell = grid_[cellKey(ix, iy)];
        if (cell.id != 0 && cell.id != u.id) {
          const double pet = stamp - cell.last;
          // Only real encroachments: the previous occupant has left the cell (pet > 0) and the paths cross
          // (following traffic in the same lane is headway, not a conflict).
          const bool crossing = std::abs(wrapAngle(cell.yaw - u.yaw)) > M_PI / 6;
          if (crossing && pet > 0.05 && pet < cfg_.pet_warning * 2) {
            const std::pair<std::uint32_t, std::uint32_t> key = std::minmax(cell.id, u.id);
            auto it = pets.find(key);
            if (it == pets.end() || pet < it->second.first) pets[key] = {pet, centre};
          }
        }
        cell.id = u.id;
        cell.last = stamp;
        cell.yaw = u.yaw;
      }
    }
  }
  if (grid_.size() > 400000) std::erase_if(grid_, [&](const auto& kv) { return stamp - kv.second.last > 10.0; });
}

std::vector<NearMissEvent> NearMissDetector::update(double stamp, const std::vector<RoadUser>& users) {
  std::vector<NearMissEvent> out;
  current_.clear();
  std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<double, Eigen::Vector2d>> pets;
  updatePetGrid(stamp, users, pets);

  std::set<std::pair<std::uint32_t, std::uint32_t>> active;
  for (size_t i = 0; i < users.size(); ++i) {
    for (size_t j = i + 1; j < users.size(); ++j) {
      const RoadUser &a = users[i], &b = users[j];
      if (a.state == TrackState::Tentative || b.state == TrackState::Tentative) continue;
      if ((a.pos - b.pos).norm() > cfg_.pair_range) continue;
      if (a.speed < 1.0 && b.speed < 1.0) continue;
      // Pedestrians walking together are not conflicts.
      if (a.cls == ObjectClass::Pedestrian && b.cls == ObjectClass::Pedestrian) continue;
      ConflictMetrics m = evaluatePair(a, b);
      const std::pair<std::uint32_t, std::uint32_t> key = std::minmax(a.id, b.id);
      if (auto it = pets.find(key); it != pets.end()) {
        m.pet = it->second.first;
        if (!m.ttc && !m.mttc) m.conflict_point = it->second.second;
      }
      const bool critical = (m.ttc && *m.ttc < cfg_.ttc_critical) || (m.mttc && *m.mttc < cfg_.ttc_critical) ||
                            (m.pet && *m.pet < cfg_.pet_critical) || m.drac > cfg_.drac_critical;
      const bool warning = critical || (m.ttc && *m.ttc < cfg_.ttc_warning) ||
                           (m.mttc && *m.mttc < cfg_.ttc_warning) || (m.pet && *m.pet < cfg_.pet_warning);
      if (m.ttc || m.mttc || m.pet) current_[key] = m;
      if (!warning) continue;
      active.insert(key);
      auto [it, inserted] = open_.try_emplace(key);
      NearMissEvent& e = it->second;
      const RoadUser& ua = a.id == key.first ? a : b;
      const RoadUser& ub = a.id == key.first ? b : a;
      const std::string prev_sev = e.severity;
      if (inserted) {
        e.event_id = next_event_++;
        e.a = key.first;
        e.b = key.second;
        e.cls_a = ua.cls;
        e.cls_b = ub.cls;
        e.type = conflictType(ua, ub);
        e.start = stamp;
        e.severity = "warning";
      }
      e.end = stamp;
      auto keepMin = [](std::optional<double>& dst, const std::optional<double>& v) {
        if (v && (!dst || *v < *dst)) dst = v;
      };
      keepMin(e.min_ttc, m.ttc);
      keepMin(e.min_mttc, m.mttc);
      keepMin(e.min_pet, m.pet);
      e.max_drac = std::max(e.max_drac, m.drac);
      e.speed_a = std::max(e.speed_a, ua.speed);
      e.speed_b = std::max(e.speed_b, ub.speed);
      e.point = m.conflict_point;
      if (critical) e.severity = "critical";
      if (cam_ && cam_->anchor.valid) std::tie(e.lat, e.lon) = enuToGeo(cam_->anchor, e.point);
      if (inserted || e.severity != prev_sev) out.push_back(e);
    }
  }
  // Close events that are no longer active (grace period to bridge single-frame drop-outs).
  for (auto it = open_.begin(); it != open_.end();) {
    if (!active.count(it->first) && stamp - it->second.end > 0.5) {
      it->second.finished = true;
      out.push_back(it->second);
      it = open_.erase(it);
    } else {
      ++it;
    }
  }
  // Hard braking.
  std::set<std::uint32_t> braking_now;
  for (const auto& u : users) {
    if (!isVehicle(u.cls) || u.state != TrackState::Confirmed) continue;
    if (u.accel < cfg_.hard_brake && u.speed > 2.0 && u.accel_sigma < 1.5) {
      braking_now.insert(u.id);
      auto [it, inserted] = braking_.try_emplace(u.id);
      NearMissEvent& e = it->second;
      if (inserted) {
        e.event_id = next_event_++;
        e.a = u.id;
        e.cls_a = u.cls;
        e.type = "hard_braking";
        e.severity = u.accel < -5.0 ? "critical" : "warning";
        e.start = stamp;
        e.speed_a = u.speed;
        e.point = u.pos;
        if (cam_ && cam_->anchor.valid) std::tie(e.lat, e.lon) = enuToGeo(cam_->anchor, e.point);
        out.push_back(e);
      }
      e.end = stamp;
      e.min_accel_a = std::min(e.min_accel_a, u.accel);
    }
  }
  for (auto it = braking_.begin(); it != braking_.end();) {
    if (!braking_now.count(it->first) && stamp - it->second.end > 0.5) {
      it->second.finished = true;
      out.push_back(it->second);
      it = braking_.erase(it);
    } else {
      ++it;
    }
  }
  return out;
}

}  // namespace jp
