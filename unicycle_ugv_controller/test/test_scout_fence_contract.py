#!/usr/bin/env python3
"""Scout planner/controller containment, not a physical stopping certificate.

Planner contract reviewed on 2026-09-18:
lxk36/academic ros1_ws/src/planner/formation_generator/config/scenarios/
{ugv4_scout_eight,ugv4_scout_eight_workshop}/scenario.yaml exploration/x_lo,x_hi.
Re-review both sides when either scene's planning box changes.
Run: python3 unicycle_ugv_controller/test/test_scout_fence_contract.py
"""
import copy
import math
from pathlib import Path
import unittest

import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config/scout_flatness.yaml"
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

    def test_all_four_faces_have_tracking_margin(self):
        self.assertTrue(contains_with_margin(self.config["fence"]))

    def test_old_equal_fence_is_rejected(self):
        self.assertFalse(contains_with_margin(PLANNING))

    def test_each_single_face_regression_is_rejected(self):
        for key in PLANNING:
            with self.subTest(face=key):
                broken = copy.deepcopy(self.config["fence"])
                broken[key] = PLANNING[key]
                self.assertFalse(contains_with_margin(broken))

    def test_nonfinite_fence_is_rejected(self):
        for value in (float("nan"), float("inf"), True):
            broken = copy.deepcopy(self.config["fence"])
            broken["x_max"] = value
            self.assertFalse(contains_with_margin(broken))

    def test_existing_control_strategy_and_authority_are_retained(self):
        self.assertEqual(self.config["tracking_strategy"], "flatness")
        self.assertEqual(self.config["state_source"], "platform_pose")
        self.assertEqual(self.config["chassis"],
                         {"max_linear_speed": 1.5, "max_yaw_rate": 1.05})
        self.assertEqual(self.config["flatness"]["kp"], 6.)
        self.assertEqual(self.config["flatness"]["kv"], 4.)
        self.assertEqual(self.config["state_timeout"], .2)


if __name__ == "__main__":
    unittest.main()
