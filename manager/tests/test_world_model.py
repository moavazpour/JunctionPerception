"""Tests of the site world model (manager/world_model.py): time alignment, association, fusion,
global IDs and the site state contract. Run: python3 -m unittest discover -s manager/tests"""

from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "manager"))

from world_model import (  # noqa: E402
    NS,
    ClassSize,
    FusionSettings,
    SiteWorldModel,
)

FIXTURE = ROOT / "tests" / "fixtures" / "site_state_v1.json"
OBSERVATION = ROOT / "tests" / "fixtures" / "camera_observation_v2.json"
SIZES = {"car": ClassSize(4.5, 1.8), "pedestrian": ClassSize(0.5, 0.5)}


def obj(track: int, x: float, y: float, vx: float = 0.0, vy: float = 0.0, **extra) -> dict:
    o = {
        "id": track, "class": "car", "vru": False, "rider": False, "x": x, "y": y,
        "vx": vx, "vy": vy, "speed_mps": math.hypot(vx, vy), "speed_std_mps": 0.2,
        "heading_rad": None, "heading_source": None, "position_cov_m2": [0.09, 0.0, 0.09],
        "confidence": 0.9, "length_m": None, "width_m": None, "validated": True,
    }
    o.update(extra)
    return o


def state(t_s: float, objects: list[dict], frame: int = 0, time_source: str = "media") -> dict:
    return {
        "schema_version": 2, "scene_id": "site", "frame": frame,
        "timestamp_ns": round(t_s * NS), "time_source": time_source, "source": "live",
        "objects": objects, "signals": [],
    }


def run(model: SiteWorldModel, feeds: dict[str, list[dict]]) -> list[dict]:
    """Feeds the states of all cameras in site-time order, as they would arrive; returns the
    steps."""
    merged = sorted(((s["timestamp_ns"] + model.feeds[c].offset_ns, c, s)
                     for c, states in feeds.items() for s in states),
                    key=lambda e: (e[0], e[1]))
    out = []
    for _, camera, s in merged:
        model.observe(camera, s)
        site_state = model.step()
        if site_state is not None:
            out.append(site_state)
    return out


def moving_car(camera_track: int, t0: float, n: int, dt: float = 0.1, x0: float = 0.0,
               speed: float = 10.0, bias: tuple[float, float] = (0.0, 0.0), delay: float = 0.0):
    """States of one camera seeing a car on y = 0 at `speed`, captured at t, t+dt, ... with a
    position bias (calibration) and a capture delay (the camera's frames are later by `delay`)."""
    states = []
    for i in range(n):
        t = t0 + i * dt + delay
        x = x0 + speed * (t - t0)
        states.append(state(t, [obj(camera_track, x + bias[0], bias[1], vx=speed)], frame=i))
    return states


class SingleCamera(unittest.TestCase):
    def test_one_road_user_per_track_and_stable_id(self):
        model = SiteWorldModel("s", ["a"], class_sizes=SIZES)
        steps = run(model, {"a": moving_car(7, 0.0, 10)})
        self.assertEqual(len(steps), 10)
        ids = {u["id"] for s in steps for u in s["road_users"]}
        self.assertEqual(ids, {1})
        last = steps[-1]
        self.assertEqual(last["fusion"]["mode"], "single")
        self.assertAlmostEqual(last["road_users"][0]["x"], 9.0, places=3)
        self.assertEqual(last["road_users"][0]["observed_by"][0]["track_id"], 7)

    def test_unknown_heading_stays_unknown(self):
        model = SiteWorldModel("s", ["a"], class_sizes=SIZES)
        steps = run(model, {"a": [state(0.0, [obj(1, 5.0, 5.0)]), state(0.1, [obj(1, 5.0, 5.0)])]})
        user = steps[-1]["road_users"][0]
        self.assertIsNone(user["heading_rad"])
        self.assertIsNone(user["heading_source"])
        # No measured footprint: the class size, marked as such.
        self.assertEqual((user["length_m"], user["size_source"]), (4.5, "class"))

    def test_measured_heading_is_kept_with_its_source(self):
        model = SiteWorldModel("s", ["a"])
        o = obj(1, 0.0, 0.0, heading_rad=1.2, heading_source="keypoints", length_m=4.1, width_m=1.7)
        steps = run(model, {"a": [state(0.0, [o])]})
        user = steps[-1]["road_users"][0]
        self.assertEqual((user["heading_rad"], user["heading_source"]), (1.2, "keypoints"))
        self.assertEqual((user["length_m"], user["size_source"]), (4.1, "measured"))

    def test_id_kept_through_a_camera_id_switch(self):
        """The camera loses the track for 0.3 s and picks it up under a new track ID."""
        model = SiteWorldModel("s", ["a"])
        states = []
        for i in range(20):
            t = i * 0.1
            if 8 <= i < 11:
                states.append(state(t, [], frame=i))
            else:
                states.append(state(t, [obj(1 if i < 8 else 2, 10 * t, 0.0, vx=10.0)], frame=i))
        steps = run(model, {"a": states})
        during = steps[9]["road_users"]
        self.assertEqual([u["status"] for u in during], ["coasting"])
        self.assertEqual({u["id"] for s in steps for u in s["road_users"]}, {1})
        self.assertEqual(steps[-1]["road_users"][0]["observed_by"][0]["track_id"], 2)
        self.assertEqual(steps[-1]["road_users"][0]["status"], "tracked")

    def test_lost_road_user_is_dropped_after_coasting(self):
        model = SiteWorldModel("s", ["a"])
        states = [state(0.0, [obj(1, 0.0, 0.0)])] + [state(0.1 * i, []) for i in range(1, 15)]
        steps = run(model, {"a": states})
        self.assertEqual(len(steps[5]["road_users"]), 1)  # 0.5 s: coasting
        self.assertEqual(steps[-1]["road_users"], [])  # 1.4 s > coast_vehicle_s


class TwoCameras(unittest.TestCase):
    def test_same_car_seen_by_both_is_one_road_user(self):
        model = SiteWorldModel("s", ["a", "b"])
        steps = run(model, {
            "a": moving_car(3, 0.0, 20, bias=(0.4, 0.2)),
            "b": moving_car(9, 0.0, 20, bias=(-0.3, -0.1)),
        })
        last = steps[-1]
        self.assertEqual(last["fusion"]["mode"], "fused")
        self.assertEqual(len(last["road_users"]), 1)
        user = last["road_users"][0]
        self.assertEqual([m["camera"] for m in user["observed_by"]], ["a", "b"])
        self.assertAlmostEqual(user["x"], 19.05, places=2)  # mean of the two biases
        self.assertEqual({u["id"] for s in steps for u in s["road_users"]}, {1})

    def test_delayed_camera_is_aligned_by_prediction(self):
        """Camera b captures 40 ms after camera a: its track is predicted to the site time
        instead of being averaged with a position of another instant."""
        model = SiteWorldModel("s", ["a", "b"])
        steps = run(model, {"a": moving_car(1, 0.0, 30), "b": moving_car(2, 0.0, 30, delay=0.04)})
        last = steps[-1]
        self.assertEqual(len(last["road_users"]), 1)
        user = last["road_users"][0]
        expected = 10.0 * last["t_ns"] / NS
        self.assertAlmostEqual(user["x"], expected, places=3)
        self.assertTrue(all(abs(m["dt_ms"]) <= 100 for m in user["observed_by"]))

    def test_known_offset_is_applied(self):
        """b's recording started 2 s after a's: with the offset configured they align."""
        model = SiteWorldModel("s", {"a": 0, "b": 2 * NS})
        b = moving_car(2, -2.0, 30)  # b's clock reads 2 s less for the same instant
        steps = run(model, {"a": moving_car(1, 0.0, 30), "b": b})
        self.assertEqual(len(steps[-1]["road_users"]), 1)

    def test_far_apart_cars_stay_apart(self):
        model = SiteWorldModel("s", ["a", "b"])
        steps = run(model, {"a": moving_car(1, 0.0, 10), "b": moving_car(1, 0.0, 10, bias=(0.0, 8.0))})
        self.assertEqual(len(steps[-1]["road_users"]), 2)

    def test_ambiguous_track_is_not_forced_into_a_road_user(self):
        """Camera a sees two standing cars 1.2 m apart; camera b sees one car right between
        them: it is not merged with either."""
        model = SiteWorldModel("s", ["a", "b"])
        a = [state(0.1 * i, [obj(1, 0.0, 0.0), obj(2, 0.0, 1.2)]) for i in range(10)]
        b = [state(0.1 * i, [obj(5, 0.0, 0.6)]) for i in range(10)]
        steps = run(model, {"a": a, "b": b})
        users = steps[-1]["road_users"]
        self.assertEqual(len(users), 3)
        self.assertTrue(all(len(u["observed_by"]) == 1 for u in users))

    def test_stale_camera_does_not_hold_back_the_site(self):
        model = SiteWorldModel("s", ["a", "b"])
        steps = run(model, {"a": moving_car(1, 0.0, 30), "b": moving_car(2, 0.0, 5)})
        last = steps[-1]
        self.assertAlmostEqual(last["t_ns"] / NS, 2.9, places=6)
        cameras = {c["id"]: c["state"] for c in last["cameras"]}
        self.assertEqual(cameras, {"a": "ok", "b": "stale"})
        self.assertEqual(len(last["road_users"]), 1)

    def test_mixed_clocks_run_unfused(self):
        model = SiteWorldModel("s", ["a", "b"])
        a = moving_car(1, 0.0, 5)
        b = [dict(s, time_source="arrival") for s in moving_car(2, 0.0, 5)]
        steps = run(model, {"a": a, "b": b})
        self.assertEqual(steps[-1]["fusion"]["mode"], "unfused")
        self.assertIsNotNone(steps[-1]["fusion"]["reason"])

    def test_fusion_can_be_switched_off(self):
        model = SiteWorldModel("s", ["a", "b"], FusionSettings(fuse_cameras=False))
        steps = run(model, {"a": moving_car(1, 0.0, 10), "b": moving_car(2, 0.0, 10)})
        self.assertEqual(steps[-1]["fusion"]["mode"], "unfused")
        self.assertEqual(len(steps[-1]["road_users"]), 2)

    def test_recordings_without_one_clock_run_unfused(self):
        model = SiteWorldModel("s", ["a", "b"], shared_media_clock=False)
        steps = run(model, {"a": moving_car(1, 0.0, 5), "b": moving_car(2, 0.0, 5)})
        self.assertEqual(steps[-1]["fusion"]["mode"], "unfused")
        self.assertEqual(len(steps[-1]["road_users"]), 2)  # shown, not merged

    def test_duplicates_from_a_late_start_are_merged(self):
        """b first sees the car 3 m off (outside the gate), then agrees with a: one road user
        after `merge_after` steps, keeping the older ID."""
        model = SiteWorldModel("s", ["a", "b"])
        b = moving_car(2, 0.0, 20)
        for s in b[:3]:
            s["objects"][0]["y"] = 3.5
        steps = run(model, {"a": moving_car(1, 0.0, 20), "b": b})
        self.assertEqual(len(steps[3]["road_users"]), 2)
        self.assertEqual([u["id"] for u in steps[-1]["road_users"]], [1])
        self.assertEqual(len(steps[-1]["road_users"][0]["observed_by"]), 2)

    def test_seek_starts_a_new_run(self):
        model = SiteWorldModel("s", ["a"])
        steps = run(model, {"a": moving_car(1, 5.0, 5)})
        model.observe("a", state(0.0, [obj(1, 0.0, 0.0)]))
        after = model.step()
        self.assertEqual((steps[-1]["run"], after["run"]), (0, 1))
        self.assertEqual(after["road_users"][0]["first_seen_ns"], 0)

    def test_deterministic(self):
        feeds = {"a": moving_car(1, 0.0, 15, bias=(0.2, 0.0)), "b": moving_car(4, 0.0, 15, delay=0.03)}
        one = run(SiteWorldModel("s", ["a", "b"]), feeds)
        two = run(SiteWorldModel("s", ["a", "b"]), json.loads(json.dumps(feeds)))
        self.assertEqual(one, two)


class Contract(unittest.TestCase):
    def test_site_state_matches_the_fixture(self):
        """The fixture is the contract the frontend tests against (web/tests/siteState.test.ts):
        the model must produce exactly its fields."""
        fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
        model = SiteWorldModel("s", ["a"], class_sizes=SIZES)
        model.observe("a", state(0.0, [obj(1, 0.0, 0.0)]))
        produced = model.step()
        self.assertEqual(set(produced), set(fixture))
        self.assertEqual(set(produced["road_users"][0]), set(fixture["road_users"][0]))
        self.assertEqual(set(produced["road_users"][0]["observed_by"][0]),
                         set(fixture["road_users"][0]["observed_by"][0]))
        self.assertEqual(set(produced["cameras"][0]), set(fixture["cameras"][0]))
        self.assertEqual(set(produced["fusion"]), set(fixture["fusion"]))
        self.assertEqual(produced["schema_version"], fixture["schema_version"])

    def test_reads_the_camera_observation_fixture(self):
        """The observation jp_server publishes (tests/fixtures/camera_observation_v2.json, which
        the C++ serializer reproduces exactly) is the model's input."""
        observation = json.loads(OBSERVATION.read_text(encoding="utf-8"))
        model = SiteWorldModel("s", ["south1"], class_sizes=SIZES)
        model.observe("south1", observation)
        users = model.step()["road_users"]
        self.assertEqual([u["observed_by"][0]["track_id"] for u in users], [17, 42])
        self.assertEqual(model.step(), None)  # nothing new
        pedestrian = users[1]
        self.assertIsNone(pedestrian["heading_rad"])
        self.assertFalse(pedestrian["validated"])
        self.assertEqual(model.clock(), "unix")  # "capture" is absolute time

    def test_rejects_unknown_camera_schema(self):
        model = SiteWorldModel("s", ["a"])
        with self.assertRaises(ValueError):
            model.observe("a", dict(state(0.0, []), schema_version=9))

    def test_settings_reject_unknown_keys(self):
        with self.assertRaises(ValueError):
            FusionSettings.from_dict({"gate": 3})


if __name__ == "__main__":
    unittest.main()
