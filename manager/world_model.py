"""Site world model: the one place where camera observations become site road users.

Every camera server (jp_server, C++) publishes per-camera observations: its own tracks with
filtered position, velocity, heading and their uncertainties, in the site's world frame
(schema: docs/architecture.md, "Camera observation"). This module aligns them in time,
associates the tracks of different cameras, fuses them and keeps stable global road-user IDs.
The result, one site state per step, is the only physical road-user state clients see
(schema: "Site state", fixture tests/fixtures/site_state_v1.json).

Pure and deterministic: no I/O, no wall clock. `SiteWorldModel.observe()` takes camera states,
`step()` returns the site state for the latest instant every fresh camera has reported. The
manager (site_feed.py) feeds it live; tools/eval_fusion.py feeds it recorded JSONL.

Time. Each camera state carries `timestamp_ns` and `time_source`:
  media    time since the start of the camera's recording (recorded videos); comparable between
           cameras only when they play by one site clock, plus the camera's configured offset;
  capture  absolute capture time (Unix ns; image sequences with timestamps);
  arrival  absolute time the server received the frame (live streams; includes the latency).
All cameras of a site must be on one clock kind ("media" or absolute) to be fused; otherwise the
site runs unfused and says so.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

SITE_STATE_SCHEMA = "site_state"
SITE_STATE_SCHEMA_VERSION = 1
CAMERA_STATE_SCHEMA_VERSIONS = (1, 2)

NS = 1_000_000_000
HEADING_PRIORITY = {"label": 4, "keypoints": 3, "motion": 2, "held": 1}


@dataclass(frozen=True)
class FusionSettings:
    """Association and lifecycle parameters (configs/manager.yaml, world_model)."""

    # A camera whose latest state is this much older than the newest camera's is stale: its
    # tracks are not used and do not hold back the site time.
    stale_after_s: float = 0.5
    # Largest prediction of a camera track to the site time; older states are not used.
    max_predict_s: float = 0.3
    # Chi-square gates (2 degrees of freedom) on the position difference with the summed
    # covariances: 99 % to associate, 99.99 % (looser) to keep an established binding.
    gate_chi2: float = 9.21
    keep_chi2: float = 18.4
    # An established binding is dropped after this many steps outside the keep gate in a row.
    unbind_after: int = 3
    # Euclidean ceilings on top of the gates (m): a very uncertain track never reaches further.
    max_distance_vehicle_m: float = 4.0
    max_distance_vru_m: float = 2.0
    max_velocity_difference_mps: float = 4.0
    # Second candidate closer than this (chi-square units) to the best: ambiguous, not merged.
    ambiguity_margin_chi2: float = 2.0
    # Two road users of different cameras that are mutually nearest within the gate for this
    # many steps in a row are one (e.g. after an ambiguous start): the younger joins the older.
    merge_after: int = 3
    # Calibration disagreement between cameras (m, one sigma), added to every pair's covariance.
    camera_bias_m: float = 0.5
    # Position sigma (m) of tracks whose source reports no covariance (ground truth, old data).
    default_position_sigma_m: float = 0.5
    # Velocity sigma (m/s) of tracks whose source reports no speed uncertainty.
    default_speed_sigma_mps: float = 1.0
    # How long a road user no camera sees is kept (coasting) before it is dropped.
    coast_vehicle_s: float = 1.0
    coast_vru_s: float = 0.6
    # Coasting road users move on with their velocity only if it is known this well (m/s).
    coast_max_speed_sigma_mps: float = 1.0
    # A camera's time going back by more than this is a seek or a loop: a new run.
    rewind_s: float = 1.0

    @staticmethod
    def from_dict(values: dict[str, Any] | None) -> FusionSettings:
        values = values or {}
        unknown = set(values) - set(FusionSettings.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown world_model settings: {', '.join(sorted(unknown))}")
        return FusionSettings(**values)


@dataclass
class ClassSize:
    length_m: float
    width_m: float


# ----------------------------------------------------------------------------- 2x2 algebra

Cov = tuple[float, float, float]  # xx, xy, yy


def cov_add(a: Cov, b: Cov) -> Cov:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def cov_inverse(c: Cov) -> Cov | None:
    det = c[0] * c[2] - c[1] * c[1]
    if det <= 1e-12:
        return None
    return (c[2] / det, -c[1] / det, c[0] / det)


def mahalanobis2(dx: float, dy: float, c: Cov) -> float:
    inverse = cov_inverse(c)
    if inverse is None:
        return math.inf
    return dx * dx * inverse[0] + 2 * dx * dy * inverse[1] + dy * dy * inverse[2]


def isotropic(sigma: float) -> Cov:
    return (sigma * sigma, 0.0, sigma * sigma)


# ----------------------------------------------------------------------------- observations


@dataclass
class Observation:
    """One camera track, predicted to the site time of the step."""

    camera: str
    track: int
    t_ns: int  # site time of the camera state it comes from
    dt_s: float  # prediction from t_ns to the step time
    obj: dict[str, Any]
    x: float
    y: float
    cov: Cov
    speed_sigma: float

    @property
    def vru(self) -> bool:
        return bool(self.obj.get("vru"))

    @property
    def key(self) -> tuple[str, int]:
        return (self.camera, self.track)


def observation(camera: str, t_ns: int, t_out: int, obj: dict[str, Any],
                settings: FusionSettings) -> Observation:
    """A camera track predicted from its capture time to `t_out` at constant velocity; the
    prediction adds the velocity uncertainty to the position covariance."""
    dt = (t_out - t_ns) / NS
    cov_raw = obj.get("position_cov_m2")
    cov: Cov = (tuple(cov_raw) if cov_raw else
                isotropic(settings.default_position_sigma_m))  # type: ignore[assignment]
    speed_sigma = obj.get("speed_std_mps")
    speed_sigma = settings.default_speed_sigma_mps if speed_sigma is None else float(speed_sigma)
    vx, vy = float(obj.get("vx", 0.0)), float(obj.get("vy", 0.0))
    growth = (speed_sigma * dt) ** 2
    return Observation(
        camera=camera, track=int(obj["id"]), t_ns=t_ns, dt_s=dt, obj=obj,
        x=float(obj["x"]) + vx * dt, y=float(obj["y"]) + vy * dt,
        cov=cov_add(cov, (growth, 0.0, growth)), speed_sigma=speed_sigma,
    )


# ----------------------------------------------------------------------------- global tracks


@dataclass
class GlobalTrack:
    """A site road user: the camera tracks bound to it and its fused state."""

    id: int
    vru: bool
    owner: str | None  # unfused mode: the one camera whose tracks it may hold
    first_seen_ns: int
    x: float = 0.0
    y: float = 0.0
    cov: Cov = (1.0, 0.0, 1.0)
    vx: float = 0.0
    vy: float = 0.0
    speed_sigma: float | None = None
    t_ns: int = 0  # time of the fused state
    last_observed_ns: int = 0
    members: list[Observation] = field(default_factory=list)
    obj: dict[str, Any] = field(default_factory=dict)  # attributes of the leading member
    heading: float | None = None
    heading_source: str | None = None
    size: tuple[float, float] | None = None
    size_measured: bool = False
    validated: bool = True

    def predict(self, t_ns: int, settings: FusionSettings) -> tuple[float, float, Cov]:
        dt = (t_ns - self.t_ns) / NS
        sigma = self.speed_sigma if self.speed_sigma is not None else settings.default_speed_sigma_mps
        moves = self.speed_sigma is not None and self.speed_sigma <= settings.coast_max_speed_sigma_mps
        x = self.x + (self.vx * dt if moves else 0.0)
        y = self.y + (self.vy * dt if moves else 0.0)
        growth = (sigma * dt) ** 2
        return x, y, cov_add(self.cov, (growth, 0.0, growth))


@dataclass
class CameraFeed:
    time_source: str = "media"
    offset_ns: int = 0
    states: list[tuple[int, dict[str, Any]]] = field(default_factory=list)  # (site t_ns, state)

    def latest_t(self) -> int | None:
        return self.states[-1][0] if self.states else None

    def at(self, t_out: int) -> tuple[int, dict[str, Any]] | None:
        """The newest state not after `t_out` (else the oldest kept)."""
        chosen = None
        for t, state in self.states:
            if t <= t_out:
                chosen = (t, state)
        return chosen or (self.states[0] if self.states else None)


def clock_kind(time_source: str) -> str:
    return "media" if time_source == "media" else "unix"


class SiteWorldModel:
    """Time alignment, association, fusion and global IDs of one site."""

    def __init__(self, site_id: str, cameras: dict[str, int] | list[str],
                 settings: FusionSettings | None = None,
                 class_sizes: dict[str, ClassSize] | None = None,
                 shared_media_clock: bool = True) -> None:
        """`cameras`: camera ids, or camera id -> time offset (ns) added to its timestamps.
        `shared_media_clock`: the cameras' recordings play by one clock (site clock or a single
        camera); without it, media timestamps of different cameras cannot be compared."""
        self.site_id = site_id
        self.settings = settings or FusionSettings()
        self.class_sizes = class_sizes or {}
        self.shared_media_clock = shared_media_clock
        offsets = cameras if isinstance(cameras, dict) else {c: 0 for c in cameras}
        self.feeds = {c: CameraFeed(offset_ns=o) for c, o in sorted(offsets.items())}
        self.run = 0
        self.seq = 0
        self.next_id = 1
        self.tracks: list[GlobalTrack] = []
        # (camera, track) -> global id, and steps outside the keep gate in a row.
        self.bindings: dict[tuple[str, int], int] = {}
        self.violations: dict[tuple[str, int], int] = {}
        # Pairs of road users (ids) agreeing as one, and for how many steps in a row.
        self.merge_counts: dict[tuple[int, int], int] = {}
        self.t_out: int | None = None
        self.seen: dict[str, int] = {}  # latest time of each camera at the last step
        self.signals: dict[str, list[dict[str, Any]]] = {}

    # ------------------------------------------------------------------ input

    def set_offset(self, camera: str, offset_ns: int) -> None:
        self.feeds.setdefault(camera, CameraFeed()).offset_ns = offset_ns

    def observe(self, camera: str, state: dict[str, Any]) -> None:
        """A camera state (observation schema v1 or v2). Older states than the camera's newest
        are dropped, except a rewind (seek, loop), which starts a new run for the site."""
        if state.get("schema_version") not in CAMERA_STATE_SCHEMA_VERSIONS:
            raise ValueError(f"unsupported camera state schema {state.get('schema_version')}")
        feed = self.feeds.setdefault(camera, CameraFeed())
        feed.time_source = str(state.get("time_source", "media"))
        t = int(state["timestamp_ns"]) + feed.offset_ns
        latest = feed.latest_t()
        if latest is not None and t <= latest:
            if latest - t > self.settings.rewind_s * NS:
                self.new_run()
                feed = self.feeds[camera]
            else:
                return
        feed.states.append((t, state))
        del feed.states[:-3]
        self.signals[camera] = [{"camera": camera, **s} for s in state.get("signals", [])]

    def new_run(self) -> None:
        """Seek, loop or restart: track IDs of the cameras start over, so do the road users."""
        self.run += 1
        self.tracks.clear()
        self.bindings.clear()
        self.violations.clear()
        self.merge_counts.clear()
        self.t_out = None
        self.seen = {}
        for feed in self.feeds.values():
            feed.states.clear()

    # ------------------------------------------------------------------ mode

    def mode(self) -> tuple[str, str | None]:
        """"single", "fused" or "unfused" (with the reason)."""
        live = [c for c, f in self.feeds.items() if f.states]
        if len(self.feeds) <= 1:
            return "single", None
        kinds = {clock_kind(self.feeds[c].time_source) for c in live}
        if len(kinds) > 1:
            return "unfused", "cameras on different clocks (recording time and live time)"
        if kinds == {"media"} and not self.shared_media_clock:
            return "unfused", "recordings do not play by one site clock"
        return "fused", None

    def clock(self) -> str:
        """What `t_ns` of the site state counts: "media" (recording time), "unix" (absolute) or
        "mixed" (unfused cameras on different clocks; `t_ns` is then only the newest time)."""
        kinds = {clock_kind(f.time_source) for f in self.feeds.values() if f.states}
        return kinds.pop() if len(kinds) == 1 else "mixed"

    # ------------------------------------------------------------------ step

    def step(self) -> dict[str, Any] | None:
        """Advances to the newest instant every fresh camera has reported and returns the site
        state, or None if that instant has not moved since the last step."""
        latest = {c: f.latest_t() for c, f in self.feeds.items() if f.states}
        if not latest:
            return None
        t_ref = max(latest.values())
        mode, reason = self.mode()
        if mode == "unfused":
            # No common time: each camera's road users at its own latest state, unaligned.
            if latest == self.seen:
                return None
            t_out = t_ref
            times = dict(latest)
        else:
            stale_ns = self.settings.stale_after_s * NS
            fresh = {c: t for c, t in latest.items() if t_ref - t <= stale_ns}
            t_out = min(fresh.values())
            if self.t_out is not None and t_out <= self.t_out:
                return None
            times = {c: t_out for c in fresh}
        self.t_out = t_out
        self.seen = dict(latest)

        observations: dict[str, list[Observation]] = {}
        used: dict[str, int] = {}
        for camera, t in times.items():
            chosen = self.feeds[camera].at(t)
            if chosen is None:
                continue
            t_cam, state = chosen
            if abs(t - t_cam) > self.settings.max_predict_s * NS:
                continue
            used[camera] = t_cam
            observations[camera] = [observation(camera, t_cam, t, o, self.settings)
                                    for o in state.get("objects", [])]

        self.associate(observations, times, t_out, fuse_cameras=mode == "fused")
        self.update_tracks(times, t_out, fuse_cameras=mode == "fused")
        self.seq += 1
        return self.site_state(t_out, t_ref, mode, reason, latest, used)

    @staticmethod
    def track_time(track: GlobalTrack, times: dict[str, int], t_out: int) -> int | None:
        """Time a road user is evaluated at: the site time, or its camera's time when unfused
        (None if that camera has no state)."""
        if track.owner is None:
            return t_out
        return times.get(track.owner)

    def associate(self, observations: dict[str, list[Observation]], times: dict[str, int],
                  t_out: int, fuse_cameras: bool) -> None:
        settings = self.settings
        for track in self.tracks:
            track.members = []
        by_id = {t.id: t for t in self.tracks}
        predicted = {}
        for track in self.tracks:
            t = self.track_time(track, times, t_out)
            predicted[track.id] = track.predict(track.t_ns if t is None else t, settings)
        alive = {o.key for obs in observations.values() for o in obs}
        for key in [k for k in self.bindings if k not in alive and k[0] in observations]:
            # The camera reported and no longer has this track.
            del self.bindings[key]
            self.violations.pop(key, None)

        for camera in sorted(observations):
            unbound: list[Observation] = []
            for o in sorted(observations[camera], key=lambda o: o.track):
                gid = self.bindings.get(o.key)
                track = by_id.get(gid) if gid is not None else None
                if track is None or track.vru != o.vru or any(m.camera == camera for m in track.members):
                    self.bindings.pop(o.key, None)
                    unbound.append(o)
                    continue
                d2 = self.distance2(o, predicted[track.id])
                if d2 > settings.keep_chi2:
                    count = self.violations.get(o.key, 0) + 1
                    if count >= settings.unbind_after:
                        del self.bindings[o.key]
                        self.violations.pop(o.key, None)
                        unbound.append(o)
                        continue
                    self.violations[o.key] = count
                else:
                    self.violations.pop(o.key, None)
                track.members.append(o)

            # Unbound tracks: re-attach to a road user (another camera's, or a coasting one) or
            # start a new one. Pairs in order of distance; ambiguous tracks are not merged.
            candidates: list[tuple[float, int, int]] = []
            for index, o in enumerate(unbound):
                costs = []
                for track in self.tracks:
                    if track.vru != o.vru or any(m.camera == camera for m in track.members):
                        continue
                    if track.owner is not None and track.owner != camera:
                        continue
                    if not fuse_cameras and track.owner is None:
                        continue
                    x, y, cov = predicted.get(track.id) or (track.x, track.y, track.cov)
                    d2 = self.gate(o, x, y, cov, track)
                    if d2 is not None:
                        costs.append((d2, track.id))
                costs.sort()
                if len(costs) > 1 and costs[1][0] - costs[0][0] < settings.ambiguity_margin_chi2:
                    continue
                candidates += [(d2, index, gid) for d2, gid in costs[:1]]
            taken: set[int] = set()
            for d2, index, gid in sorted(candidates):
                if index in taken:
                    continue
                track = by_id[gid]
                if any(m.camera == camera for m in track.members):
                    continue
                taken.add(index)
                track.members.append(unbound[index])
                self.bindings[unbound[index].key] = gid
            for index, o in enumerate(unbound):
                if index in taken:
                    continue
                track = GlobalTrack(id=self.next_id, vru=o.vru,
                                    owner=None if fuse_cameras else camera,
                                    first_seen_ns=t_out)
                self.next_id += 1
                track.members.append(o)
                self.tracks.append(track)
                by_id[track.id] = track
                predicted[track.id] = (o.x, o.y, o.cov)
                self.bindings[o.key] = track.id

    def distance2(self, o: Observation, prediction: tuple[float, float, Cov]) -> float:
        x, y, cov = prediction
        bias = isotropic(self.settings.camera_bias_m)
        return mahalanobis2(o.x - x, o.y - y, cov_add(cov_add(o.cov, cov), bias))

    def gate(self, o: Observation, x: float, y: float, cov: Cov, track: GlobalTrack) -> float | None:
        """Squared Mahalanobis distance if `o` may be the road user `track`, else None."""
        s = self.settings
        limit = s.max_distance_vru_m if o.vru else s.max_distance_vehicle_m
        if math.hypot(o.x - x, o.y - y) > limit:
            return None
        if track.members:
            # Created or re-attached in this step: its velocity is that of its member.
            lead = track.members[0]
            tvx, tvy = float(lead.obj.get("vx", 0.0)), float(lead.obj.get("vy", 0.0))
            track_sigma = lead.speed_sigma
        else:
            tvx, tvy = track.vx, track.vy
            track_sigma = track.speed_sigma or s.default_speed_sigma_mps
        dv = math.hypot(float(o.obj.get("vx", 0.0)) - tvx, float(o.obj.get("vy", 0.0)) - tvy)
        sigma_v = math.hypot(o.speed_sigma, track_sigma)
        if dv > s.max_velocity_difference_mps + 2 * min(sigma_v, s.max_velocity_difference_mps):
            return None
        d2 = self.distance2(o, (x, y, cov))
        return d2 if d2 <= s.gate_chi2 else None

    def update_tracks(self, times: dict[str, int], t_out: int, fuse_cameras: bool) -> None:
        s = self.settings
        for track in self.tracks:
            if track.members:
                t = self.track_time(track, times, t_out)
                self.fuse(track, t_out if t is None else t)
        if fuse_cameras:
            self.merge_duplicates()
        kept: list[GlobalTrack] = []
        for track in self.tracks:
            if track.members:
                kept.append(track)
                continue
            t = self.track_time(track, times, t_out)
            coast = s.coast_vru_s if track.vru else s.coast_vehicle_s
            if t is None or (t - track.last_observed_ns) / NS > coast:
                continue  # lost, or its (unfused) camera is gone
            x, y, cov = track.predict(t, s)
            track.x, track.y, track.cov, track.t_ns = x, y, cov, t
            if track.heading is not None:
                track.heading_source = "held"
            kept.append(track)
        self.tracks = kept
        live = {t.id for t in kept}
        for key in [k for k, gid in self.bindings.items() if gid not in live]:
            del self.bindings[key]
            self.violations.pop(key, None)

    def merge_duplicates(self) -> None:
        """Joins road users that turn out to be one: seen by different cameras, mutually nearest
        within the gate and moving alike for `merge_after` steps in a row."""
        s = self.settings
        active = [t for t in self.tracks if t.members and t.owner is None]
        bias = isotropic(s.camera_bias_m)
        pairs: dict[tuple[int, int], float] = {}
        partners: dict[int, list[tuple[float, int]]] = {}
        for i, a in enumerate(active):
            cameras_a = {m.camera for m in a.members}
            for b in active[i + 1:]:
                if a.vru != b.vru or cameras_a & {m.camera for m in b.members}:
                    continue
                limit = s.max_distance_vru_m if a.vru else s.max_distance_vehicle_m
                if math.hypot(a.x - b.x, a.y - b.y) > limit:
                    continue
                if math.hypot(a.vx - b.vx, a.vy - b.vy) > s.max_velocity_difference_mps:
                    continue
                d2 = mahalanobis2(a.x - b.x, a.y - b.y, cov_add(cov_add(a.cov, b.cov), bias))
                if d2 > s.gate_chi2:
                    continue
                pairs[(a.id, b.id)] = d2
                partners.setdefault(a.id, []).append((d2, b.id))
                partners.setdefault(b.id, []).append((d2, a.id))

        def clear_best(one: int, other: int) -> bool:
            """`other` is the nearest partner of `one`, and no other comes close to it."""
            ranked = sorted(partners[one])
            return ranked[0][1] == other and (
                len(ranked) == 1 or ranked[1][0] - ranked[0][0] >= s.ambiguity_margin_chi2)

        agreed = [p for p in pairs if clear_best(p[0], p[1]) and clear_best(p[1], p[0])]
        self.merge_counts = {p: self.merge_counts.get(p, 0) + 1 for p in agreed}
        by_id = {t.id: t for t in self.tracks}
        removed: set[int] = set()
        for (a_id, b_id), count in sorted(self.merge_counts.items()):
            if count < s.merge_after or a_id in removed or b_id in removed:
                continue
            keep, drop = by_id[a_id], by_id[b_id]  # a_id < b_id: the older one stays
            keep.members += drop.members
            for m in drop.members:
                self.bindings[m.key] = keep.id
                self.violations.pop(m.key, None)
            keep.first_seen_ns = min(keep.first_seen_ns, drop.first_seen_ns)
            self.fuse(keep, keep.t_ns)
            removed.add(drop.id)
        self.tracks = [t for t in self.tracks if t.id not in removed]
        self.merge_counts = {p: c for p, c in self.merge_counts.items() if not set(p) & removed}

    def fuse(self, track: GlobalTrack, t_out: int) -> None:
        """Fused state of the bound camera tracks: positions and velocities weighted by their
        inverse covariances (the cameras measure independently); attributes from the most
        confident member; the heading from the best-measured member, never averaged."""
        members = sorted(track.members, key=lambda m: (-float(m.obj.get("confidence", 0.0)), m.camera))
        info = (0.0, 0.0, 0.0)
        ix = iy = 0.0
        wv = vx = vy = 0.0
        for m in members:
            inverse = cov_inverse(m.cov) or cov_inverse(isotropic(self.settings.default_position_sigma_m))
            assert inverse is not None
            info = cov_add(info, inverse)
            ix += inverse[0] * m.x + inverse[1] * m.y
            iy += inverse[1] * m.x + inverse[2] * m.y
            w = 1.0 / max(m.speed_sigma, 0.05) ** 2
            wv += w
            vx += w * float(m.obj.get("vx", 0.0))
            vy += w * float(m.obj.get("vy", 0.0))
        cov = cov_inverse(info)
        assert cov is not None
        track.cov = cov
        track.x = cov[0] * ix + cov[1] * iy
        track.y = cov[1] * ix + cov[2] * iy
        track.vx, track.vy = vx / wv, vy / wv
        track.speed_sigma = math.sqrt(1.0 / wv)
        track.t_ns = track.last_observed_ns = t_out
        lead = members[0]
        track.obj = dict(lead.obj)
        track.validated = any(bool(m.obj.get("validated", True)) for m in members)

        headed = [m for m in members if m.obj.get("heading_rad") is not None]
        if headed:
            best = max(headed, key=lambda m: (HEADING_PRIORITY.get(str(m.obj.get("heading_source")), 0),
                                              float(m.obj.get("confidence", 0.0))))
            track.heading = float(best.obj["heading_rad"])
            track.heading_source = best.obj.get("heading_source")
        elif track.heading is not None:
            track.heading_source = "held"
        measured = [m for m in members if m.obj.get("length_m") and m.obj.get("width_m")]
        if measured:
            track.size = (float(measured[0].obj["length_m"]), float(measured[0].obj["width_m"]))
            track.size_measured = True
        elif not track.size_measured:
            size = self.class_sizes.get(str(lead.obj.get("class")))
            track.size = (size.length_m, size.width_m) if size else None

    # ------------------------------------------------------------------ output

    def site_state(self, t_out: int, t_ref: int, mode: str, reason: str | None,
                   latest: dict[str, int], used: dict[str, int]) -> dict[str, Any]:
        cameras = []
        for camera, feed in self.feeds.items():
            t = latest.get(camera)
            last = feed.states[-1][1] if feed.states else None
            cameras.append({
                "id": camera,
                "state": "offline" if t is None else ("ok" if camera in used else "stale"),
                "time_source": feed.time_source if feed.states else None,
                "offset_ms": feed.offset_ns / 1e6,
                "last_t_ns": t,
                "lag_ms": None if t is None else (t_ref - t) / 1e6,
                "frame": None if last is None else int(last.get("frame", 0)),
                "objects": None if last is None else len(last.get("objects", [])),
            })
        return {
            "schema": SITE_STATE_SCHEMA,
            "schema_version": SITE_STATE_SCHEMA_VERSION,
            "site_id": self.site_id,
            "seq": self.seq,
            "run": self.run,
            "t_ns": t_out,
            "clock": self.clock(),
            "coordinate_frame": "site",
            "fusion": {"mode": mode, "reason": reason},
            "cameras": cameras,
            "road_users": [self.road_user(t, t_out) for t in sorted(self.tracks, key=lambda t: t.id)],
            "signals": [s for c in sorted(self.signals) for s in self.signals[c]],
        }

    def road_user(self, track: GlobalTrack, t_out: int) -> dict[str, Any]:
        o = track.obj
        speed = math.hypot(track.vx, track.vy)
        size = track.size
        return {
            "id": track.id,
            "class": o.get("class", "other"),
            "vru": track.vru,
            "rider": bool(o.get("rider", False)),
            "confidence": float(o.get("confidence", 0.0)),
            "status": "tracked" if track.members else "coasting",
            "x": round(track.x, 3),
            "y": round(track.y, 3),
            "position_cov_m2": [round(v, 4) for v in track.cov],
            "vx": round(track.vx, 3),
            "vy": round(track.vy, 3),
            "speed_mps": round(speed, 3),
            "speed_std_mps": None if track.speed_sigma is None else round(track.speed_sigma, 3),
            "heading_rad": None if track.heading is None else round(track.heading, 4),
            "heading_source": track.heading_source if track.heading is not None else None,
            "length_m": None if size is None else size[0],
            "width_m": None if size is None else size[1],
            "size_source": None if size is None else ("measured" if track.size_measured else "class"),
            "validated": track.validated,
            "first_seen_ns": track.first_seen_ns,
            "last_observed_ns": track.last_observed_ns,
            "observed_by": [
                {"camera": m.camera, "track_id": m.track, "t_ns": m.t_ns,
                 "dt_ms": round(m.dt_s * 1000, 1)}
                for m in sorted(track.members, key=lambda m: m.camera)
            ],
        }
