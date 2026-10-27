#include "jp/scene/intersection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace jp {

std::optional<double> IntersectionModel::timeToDistance(double d, double v, double a) {
  if (d <= 0) return 0.0;
  if (std::abs(a) < 0.05) {
    if (v < 0.3) return std::nullopt;
    return d / v;
  }
  const double disc = v * v + 2 * a * d;
  if (disc < 0) return std::nullopt;  // decelerates to a stop before covering d
  const double t = (-v + std::sqrt(disc)) / a;
  if (t < 0 || !std::isfinite(t)) return std::nullopt;
  return t;
}

RoadUser IntersectionModel::describe(const Track& t, double stamp) const {
  RoadUser u;
  u.id = t.id;
  u.cls = t.cls;
  u.state = t.state;
  u.pos = t.position();
  u.yaw = t.yaw();
  u.speed = t.speed();
  u.accel = t.accel();
  u.yaw_rate = t.yawRate();
  u.pos_sigma = t.posSigma();
  u.speed_sigma = t.speedSigma();
  u.accel_sigma = t.accelSigma();
  u.dims = t.dims;
  u.age = stamp - t.first_seen;
  u.confidence = t.score;
  u.range = u.pos.norm();
  if (cam_ && cam_->anchor.valid) std::tie(u.lat, u.lon) = enuToGeo(cam_->anchor, u.pos);
  return u;
}

void IntersectionModel::stabilize(RoadUser& u, const Track& t, double stamp) {
  Motion& m = motion_[u.id];
  const double dt = m.last < 0 ? 0.0 : std::clamp(stamp - m.last, 0.0, 1.0);
  m.last = stamp;
  const bool vehicle = isVehicle(u.cls);
  // Direction of motion (sign of the filtered speed), only when the speed is clearly above its noise.
  const double v = t.speed(), sv = t.speedSigma();
  const double vmin = std::max(vehicle ? 1.5 : 0.7, 2.0 * sv);
  std::optional<double> motion;
  if (std::abs(v) > vmin) motion = wrapAngle(v >= 0 ? t.yaw() : t.yaw() + M_PI);
  // Direction of the lane the vehicle is on (drawn or learned): the lane closest in direction.
  std::optional<double> lane_dir;
  if (vehicle) {
    const std::optional<double> ref = motion ? motion : (m.init ? std::optional<double>(m.heading) : std::nullopt);
    double best = 1e9;
    for (const auto& l : map_.lanes) {
      const auto p = l.centerline.project(u.pos);
      if (std::abs(p.lateral) > l.width / 2 + 0.6 || (p.s <= 0.01 && p.dist > 1.0)) continue;
      const double c = ref ? std::abs(wrapAngle(p.heading - *ref)) : std::abs(p.lateral);
      if (c < best) best = c, lane_dir = p.heading;
    }
  }
  double target;
  std::string src;
  if (lane_dir && (!motion || std::abs(wrapAngle(*motion - *lane_dir)) < M_PI / 2)) {
    target = *lane_dir;
    src = "lane";
  } else if (motion) {
    target = *motion;
    src = "motion";
  } else {
    target = m.init ? m.heading : t.yaw();  // standing, unknown: hold (first sight: box orientation)
    src = "hold";
  }
  if (!m.init) {
    m.heading = target;
    m.init = true;
  } else {
    double d = wrapAngle(target - m.heading);
    if (std::abs(d) > 2.0) {
      // A reversal needs persistent evidence; a single noisy update is ignored.
      if (m.reverse_since < 0) m.reverse_since = stamp;
      if (stamp - m.reverse_since >= 1.5) m.heading = target, m.reverse_since = -1;
    } else {
      m.reverse_since = -1;
      const double rate = vehicle ? 0.8 : 3.0;  // rad/s
      m.heading = wrapAngle(m.heading + std::clamp(d, -rate * dt, rate * dt));
    }
  }
  // Speed and acceleration along the heading, smoothed (0.5 s and 1 s time constants).
  const double vabs = std::abs(v), a = (v >= 0 ? 1.0 : -1.0) * t.accel();
  const double ks = dt > 0 ? 1 - std::exp(-dt / 0.5) : 1.0, ka = dt > 0 ? 1 - std::exp(-dt / 1.0) : 1.0;
  m.speed += ks * (vabs - m.speed);
  m.accel += ka * (a - m.accel);
  u.yaw = m.heading;
  u.speed = m.speed;
  u.accel = m.accel;
  u.speed_valid = u.age >= 1.0 && sv < std::max(1.5, 0.3 * vabs);
  u.heading_valid = src == "lane" || (src == "motion" && u.speed_valid);
  u.heading_source = src;
}

void IntersectionModel::assignLane(RoadUser& u, const Track& t) {
  const Lane* best = nullptr;
  Polyline::Projection best_p;
  double best_cost = 1e18;
  const bool moving = u.speed > 1.0;
  for (const auto& l : map_.lanes) {
    const auto p = l.centerline.project(u.pos);
    if (std::abs(p.lateral) > l.width / 2 + 0.6) continue;
    if (p.s <= 0.01 && p.dist > 1.0) continue;
    const double dh = std::abs(wrapAngle(u.yaw - p.heading));
    // Moving road users must travel along the lane; parked or queued ones only need to be inside it.
    if (moving && dh > M_PI / 4 && isVehicle(u.cls)) continue;
    double cost = std::abs(p.lateral) + (moving ? dh : 0.0);
    auto it = lane_memory_.find(u.id);
    if (it != lane_memory_.end() && it->second == l.id) cost -= 0.8;  // hysteresis
    if (cost < best_cost) {
      best_cost = cost;
      best = &l;
      best_p = p;
    }
  }
  if (!best || u.cls == ObjectClass::Pedestrian) return;
  u.lane = best->id;
  u.lane_s = best_p.s;
  u.lateral = best_p.lateral;
  lane_memory_[u.id] = best->id;
  if (best->to_junction && best->stop_s) {
    const double front = u.lane_s + u.dims[0] / 2;
    const double d = *best->stop_s - front;
    u.dist_to_stop = d;
    if (d > 0) {
      u.eta = timeToDistance(d, u.speed, u.accel);
      if (u.speed > 0.3) u.eta_cv = d / u.speed;
      if (!u.eta && u.accel < -0.05) {
        u.stopping = true;
        u.stop_position_s = u.lane_s + u.speed * u.speed / (2 * -u.accel);
      }
      // First-order uncertainty of the ETA from speed and acceleration uncertainty.
      if (u.eta) {
        const double h = 1e-3;
        auto f = [&](double v, double a) { return timeToDistance(d, v, a).value_or(*u.eta); };
        const double dv = (f(u.speed + h, u.accel) - *u.eta) / h;
        const double da = (f(u.speed, u.accel + h) - *u.eta) / h;
        u.eta_sigma = std::sqrt(dv * dv * u.speed_sigma * u.speed_sigma + da * da * u.accel_sigma * u.accel_sigma);
      }
    } else {
      u.eta = 0.0;
    }
  }
  (void)t;
}

void IntersectionModel::update(double stamp, const std::vector<Track>& tracks,
                               const std::function<bool(const Eigen::Vector2d&)>& is_occluded) {
  users_.clear();
  crossings_.clear();
  std::set<std::uint32_t> alive;
  for (const auto& t : tracks) {
    if (t.state == TrackState::Tentative) continue;
    RoadUser u = describe(t, stamp);
    stabilize(u, t, stamp);
    u.occluded = t.state == TrackState::Coasting && is_occluded && is_occluded(u.pos);
    assignLane(u, t);
    alive.insert(u.id);
    // Stop line crossing events (sign change of the distance).
    if (u.dist_to_stop) {
      auto it = prev_dist_.find(u.id);
      if (it != prev_dist_.end() && it->second > 0 && *u.dist_to_stop <= 0) {
        crossings_.push_back({u.id, u.lane, stamp, u.speed, u.accel, u.cls});
        crossing_count_[u.lane]++;
      }
      prev_dist_[u.id] = *u.dist_to_stop;
    }
    users_.push_back(u);
  }
  std::erase_if(prev_dist_, [&](const auto& kv) { return !alive.count(kv.first); });
  std::erase_if(lane_memory_, [&](const auto& kv) { return !alive.count(kv.first); });
  std::erase_if(motion_, [&](const auto& kv) { return !alive.count(kv.first); });

  // Queues: slow vehicles contiguous from the stop line.
  lane_states_.clear();
  for (const auto& l : map_.lanes) {
    LaneState ls;
    ls.id = l.id;
    ls.crossings = crossing_count_[l.id];
    std::vector<RoadUser*> on;
    double vs = 0;
    for (auto& u : users_)
      if (u.lane == l.id) {
        on.push_back(&u);
        vs += u.speed;
      }
    ls.users = static_cast<int>(on.size());
    ls.mean_speed = on.empty() ? 0 : vs / on.size();
    if (l.to_junction && l.stop_s) {
      std::sort(on.begin(), on.end(), [](auto* a, auto* b) { return a->lane_s > b->lane_s; });
      double head = *l.stop_s;
      for (auto* u : on) {
        if (!u->dist_to_stop || *u->dist_to_stop < -2.0) continue;
        const double front = u->lane_s + u->dims[0] / 2;
        if (u->speed > 1.5 || head - front > 15.0) break;
        u->queued = true;
        ls.queue_count++;
        head = u->lane_s - u->dims[0] / 2;
        ls.queue_length = *l.stop_s - head;
      }
    }
    lane_states_.push_back(ls);
  }

  // Virtual detectors.
  zones_.clear();
  for (const auto& z : map_.zones) {
    ZoneState zs;
    zs.id = z.id;
    zs.type = z.type;
    zs.lane = z.lane;
    double vs = 0;
    for (const auto& u : users_) {
      if (z.type != "crosswalk" && !isVehicle(u.cls)) continue;
      const OrientedBox fb{u.pos, u.yaw, u.dims[0], u.dims[1]};
      bool inside = pointInPolygon(z.polygon, u.pos);
      if (!inside)
        for (const auto& c : fb.corners())
          if (pointInPolygon(z.polygon, c)) {
            inside = true;
            break;
          }
      if (inside) {
        zs.users.push_back(u.id);
        vs += u.speed;
      }
    }
    zs.occupied = !zs.users.empty();
    zs.mean_speed = zs.users.empty() ? 0 : vs / zs.users.size();
    auto& prev = zone_prev_[z.id];
    for (auto id : zs.users)
      if (std::find(prev.begin(), prev.end(), id) == prev.end()) zs.entered_now++;
    prev = zs.users;
    zone_count_[z.id] += zs.entered_now;
    zs.count = zone_count_[z.id];
    auto& h = zone_hist_[z.id];
    h.emplace_back(stamp, zs.occupied);
    while (!h.empty() && stamp - h.front().first > 60.0) h.pop_front();
    zs.occupancy = std::count_if(h.begin(), h.end(), [](auto& e) { return e.second; }) / std::max<double>(h.size(), 1);
    zones_.push_back(zs);
  }
}

}  // namespace jp
