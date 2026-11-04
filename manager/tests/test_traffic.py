"""Tests of the lane traffic (manager/traffic.py): lanes from camera regions, queue, passing
vehicles counted once per global ID, and the traffic block of the site state contract.
Run: python3 -m unittest discover -s manager/tests"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "manager"))

from traffic import (  # noqa: E402
    Lane,
    LaneTraffic,
    TrafficSettings,
    ground_homography,
    inside,
    lanes_from_regions,
    to_ground,
)

FIXTURE = ROOT / "tests" / "fixtures" / "site_state_v1.json"

# TUMTraf south2 as fitted by the console: normalised to h33 = 1, negative w on the ground.
SOUTH2 = [
    [0.05541632877628101, -0.27562359212014115, 140.54657655162288],
    [-0.12095883376345197, 0.04796235951105448, 196.17497813342987],
    [-0.0001018849973311642, -0.012981938854761793, 1.0],
]
# Lane 0 <= x <= 20 m, -2 <= y <= 2 m, traffic in +x.
LANE = Lane("cam/lane_1", "cam", "Lane 1", ((0.0, -2.0), (20.0, -2.0), (20.0, 2.0), (0.0, 2.0)),
            (1.0, 0.0))


def user(uid: int, x: float, y: float = 0.0, speed: float = 5.0, cls: str = "car",
         vru: bool = False) -> dict:
    return {"id": uid, "class": cls, "vru": vru, "x": x, "y": y, "speed_mps": speed}


def state(t: float, users: list[dict], run: int = 0) -> dict:
    return {"run": run, "t_ns": int(t * 1e9), "road_users": users}


def lane_of(block: dict) -> dict:
    return block["lanes"][0]


class LanesFromRegions(unittest.TestCase):
    def test_projects_lanes_of_a_homography_negative_on_the_ground(self):
        h = ground_homography(SOUTH2, (1920, 1200))
        self.assertIsNotNone(to_ground(h, 960, 900))
        self.assertIsNone(to_ground(h, 0, 0))  # sky
        regions = {"lines": [
            # Closed lane outline (first point repeated) with a direction.
            {"id": "l1", "name": "North", "points": [[800, 700], [1100, 700], [1100, 1000],
                                                     [800, 1000], [800, 700]],
             "direction": [[950, 980], [950, 720]]},
            {"id": "l2", "points": [[800, 700], [1100, 700]]},  # a line, not a lane
            {"id": "l3", "points": [[0, 0], [100, 0], [100, 50]]},  # above the horizon
            {"id": "l4", "points": [[800, 700], [1100, 700], [1100, 1000]], "direction": None},
        ]}
        lanes = lanes_from_regions("south2", regions, SOUTH2, (1920, 1200))
        self.assertEqual([lane.id for lane in lanes], ["south2/l1", "south2/l4"])
        north, unset = lanes
        self.assertEqual(north.name, "North")
        self.assertEqual(len(north.polygon), 4)
        self.assertIsNone(unset.direction)
        # The direction is the ground image of the arrow, a unit vector.
        tail = to_ground(h, 950, 980)
        head = to_ground(h, 950, 720)
        assert north.direction is not None and tail is not None and head is not None
        self.assertAlmostEqual(north.direction[0] ** 2 + north.direction[1] ** 2, 1.0)
        self.assertGreater((head[0] - tail[0]) * north.direction[0]
                           + (head[1] - tail[1]) * north.direction[1], 0)
        # Points of the image inside the outline are inside the projected lane.
        centre = to_ground(h, 950, 850)
        assert centre is not None
        self.assertTrue(inside(north.polygon, *centre))


class LaneMeasures(unittest.TestCase):
    def test_counts_a_vehicle_passing_in_the_lane_direction_once(self):
        traffic = LaneTraffic([LANE])
        for i, x in enumerate([-3.0, 1.0, 6.0, 12.0, 19.0, 23.0, 30.0]):
            block = traffic.update(state(i * 0.5, [user(7, x)]))
        self.assertEqual(lane_of(block)["passed"], 1)
        self.assertEqual(lane_of(block)["passed_by_class"], {"car": 1})
        # Back into the lane and out again with the same ID: still once.
        for i, x in enumerate([1.0, 10.0, 25.0]):
            block = traffic.update(state(10 + i * 0.5, [user(7, x)]))
        self.assertEqual(lane_of(block)["passed"], 1)

    def test_ignores_the_wrong_direction_short_moves_and_pedestrians(self):
        traffic = LaneTraffic([LANE])
        for i, x in enumerate([22.0, 15.0, 5.0, -3.0]):  # against the direction
            block = traffic.update(state(i, [user(1, x), user(2, 10.0 + i * 0.5),
                                             user(3, 2.0 + i * 5, cls="pedestrian", vru=True)]))
        block = traffic.update(state(5, []))  # 2 vanished after 1.5 m along the lane
        self.assertEqual(lane_of(block)["passed"], 0)

    def test_counts_a_vehicle_that_disappears_after_driving_through(self):
        traffic = LaneTraffic([LANE])
        traffic.update(state(0, [user(4, 2.0)]))
        traffic.update(state(1, [user(4, 12.0)]))
        block = traffic.update(state(2, []))  # out of every camera's view inside the lane
        self.assertEqual(lane_of(block)["passed"], 1)

    def test_queue_is_the_slow_vehicles_in_the_lane(self):
        traffic = LaneTraffic([LANE])
        block = traffic.update(state(0, [
            user(1, 18.0, speed=0.0), user(2, 12.0, speed=0.8), user(3, 5.0, speed=6.0),
            user(4, 30.0, speed=0.0),  # outside
            user(5, 15.0, y=1.0, speed=0.0, cls="pedestrian", vru=True),
        ]))
        lane = lane_of(block)
        self.assertEqual((lane["vehicles"], lane["queue"]), (3, 2))
        self.assertEqual(block["present_by_class"], {"car": 4, "pedestrian": 1})

    def test_without_a_direction_nothing_passes_but_the_queue_is_measured(self):
        traffic = LaneTraffic([Lane("c/l", "c", "L", LANE.polygon, None)])
        traffic.update(state(0, [user(1, 1.0, speed=0.0)]))
        traffic.update(state(1, [user(1, 19.0)]))
        block = traffic.update(state(2, [user(1, 30.0)]))
        self.assertFalse(lane_of(block)["direction_set"])
        self.assertEqual(lane_of(block)["passed"], 0)

    def test_a_new_run_starts_the_counts_over(self):
        traffic = LaneTraffic([LANE])
        for i, x in enumerate([1.0, 12.0, 25.0]):
            block = traffic.update(state(i, [user(1, x)]))
        self.assertEqual(lane_of(block)["passed"], 1)
        block = traffic.update(state(0.2, [user(1, 1.0)], run=1))  # seek: IDs start over
        self.assertEqual((lane_of(block)["passed"], block["since_ns"]), (0, 200_000_000))

    def test_settings_reject_unknown_keys(self):
        with self.assertRaises(ValueError):
            TrafficSettings.from_dict({"queue_speed": 1})
        self.assertEqual(TrafficSettings.from_dict({"min_travel_m": 5}).min_travel_m, 5.0)

    def test_traffic_block_matches_the_fixture(self):
        """The site state's "traffic" block (web/src/net/siteState.ts tests the same fixture)."""
        fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))["traffic"]
        block = LaneTraffic([LANE]).update(state(0, [user(1, 5.0)]))
        self.assertEqual(set(block), set(fixture))
        self.assertEqual(set(lane_of(block)), set(fixture["lanes"][0]))


if __name__ == "__main__":
    unittest.main()
