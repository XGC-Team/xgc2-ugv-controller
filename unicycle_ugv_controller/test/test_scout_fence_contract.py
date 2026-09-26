#!/usr/bin/env python3
"""Optional Scout proposal and production Session boundary precedence.

Planner contract reviewed on 2026-09-18:
lxk36/academic ros1_ws/src/planner/formation_generator/config/scenarios/
{ugv4_scout_eight,ugv4_scout_eight_workshop}/scenario.yaml exploration/x_lo,x_hi.
Re-review both sides when either scene's planning box changes.
Run: python3 unicycle_ugv_controller/test/test_scout_fence_contract.py
"""
import copy
import math
import sys
import tempfile
from pathlib import Path
import unittest

import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "unicycle_ugv_controller/config/scout_flatness.yaml"
CANDIDATE = CONFIG.parent / "candidates/scout_fence_margin.yaml"
sys.path.insert(0, str(REPO / "ugv_reset_safety/scripts"))
import launch_control_with_slot_poses as launch
PLANNING = {"x_min": -12., "x_max": 12., "y_min": -7., "y_max": 7.}
MARGIN = .5


def contains_with_margin(fence):
    values = [fence[key] for key in PLANNING]
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) for v in values):
        return False
    return all(fence[axis + "_min"] <= PLANNING[axis + "_min"] - MARGIN
               and fence[axis + "_max"] >= PLANNING[axis + "_max"] + MARGIN
               for axis in ("x", "y"))


class ScoutFenceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with CONFIG.open() as stream:
            cls.config = yaml.safe_load(stream)
        cls.candidate = yaml.safe_load(CANDIDATE.read_text())

    def test_all_four_faces_have_tracking_margin(self):
        self.assertTrue(contains_with_margin(self.candidate["fence"]))

    def test_old_equal_fence_is_rejected(self):
        self.assertFalse(contains_with_margin(PLANNING))

    def test_each_single_face_regression_is_rejected(self):
        for key in PLANNING:
            with self.subTest(face=key):
                broken = copy.deepcopy(self.candidate["fence"])
                broken[key] = PLANNING[key]
                self.assertFalse(contains_with_margin(broken))

    def test_nonfinite_fence_is_rejected(self):
        for value in (float("nan"), float("inf"), True):
            broken = copy.deepcopy(self.candidate["fence"])
            broken["x_max"] = value
            self.assertFalse(contains_with_margin(broken))

    def test_existing_control_strategy_and_authority_are_retained(self):
        self.assertEqual(self.config["tracking_strategy"], "flatness")
        self.assertEqual(self.config["state_source"], "platform_pose")
        self.assertEqual(self.config["chassis"],
                         {"max_linear_speed": 1.5, "max_yaw_rate": 1.05})
        self.assertEqual(self.config["flatness"]["kp"], 6.)
        self.assertEqual(self.config["flatness"]["kv"], 4.)
        self.assertEqual(self.config["state_timeout"], .5)


    def test_default_fence_is_unchanged_and_candidate_is_not_a_launch_default(self):
        self.assertEqual(self.config["fence"], PLANNING)
        for file in REPO.rglob("*.launch"):
            self.assertNotIn(CANDIDATE.name, file.read_text())

    def test_frozen_session_overrides_candidate_without_expanding_the_venue(self):
        boundary = {"schemaVersion": 1, "frameId": "world", "unit": "m",
                    "controlBounds": {"xMin": -7.5, "xMax": 7.5, "yMin": -5., "yMax": 5., "zMin": 0., "zMax": 3.},
                    "groundZ": 0.}
        poses = {"ugv1": {"reset_initial_x": 0., "reset_initial_y": 0., "reset_initial_yaw": 0.}}
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            proposed = dict(self.config, **self.candidate)
            source = root / "proposed.yaml"
            source.write_text(yaml.safe_dump(proposed))
            args = launch.materialize_controller_configs(poses, {"ugv1": source}, root / "run", boundary)
            resolved = yaml.safe_load(Path(args[0].split(":=", 1)[1]).read_text())
            self.assertEqual(resolved["fence"], {"x_min": -7.5, "x_max": 7.5, "y_min": -5., "y_max": 5.})
            self.assertFalse(contains_with_margin(resolved["fence"]))
            self.assertEqual(resolved["flatness"], self.config["flatness"])
            manifest = yaml.safe_load((root / "run/manifest.yaml").read_text())["ugv1"]
            self.assertEqual(manifest["fenceSource"], "experiment-controlBounds")
            self.assertEqual(manifest["worldBoundary"], boundary)
            launch.materialize_controller_configs(poses, {"ugv1": CONFIG}, root / "default", None)
            default = yaml.safe_load((root / "default/ugv1.yaml").read_text())
            self.assertEqual(default["fence"], PLANNING)


if __name__ == "__main__":
    unittest.main()
