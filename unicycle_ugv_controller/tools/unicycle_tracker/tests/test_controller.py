#!/usr/bin/env python3
import math
import random
import unittest

from unicycle_tracker.controller import (
    PoseVelocityFilter,
    box_saturate,
    flatness_command,
    max_box_on_plan,
    plan_reset,
    update_pose_velocity_filter,
)


class UnicycleTrackerTests(unittest.TestCase):
    def test_box_saturate_is_chassis_seventy_percent(self) -> None:
        v, w = box_saturate(2.0, -2.0)
        self.assertAlmostEqual(v, 1.05)
        self.assertAlmostEqual(w, -1.05)

    def test_filter_rejects_nonpositive_dt(self) -> None:
        filt = PoseVelocityFilter()
        self.assertFalse(update_pose_velocity_filter(filt, 1.0, 0.0))
        self.assertFalse(update_pose_velocity_filter(filt, 1.0, 1.0e-6))

    def test_filter_uses_this_sample_dt(self) -> None:
        filt = PoseVelocityFilter()
        self.assertFalse(update_pose_velocity_filter(filt, 0.0, 0.01))
        self.assertTrue(update_pose_velocity_filter(filt, 0.1, 0.01))
        slow = PoseVelocityFilter()
        update_pose_velocity_filter(slow, 0.0, 0.05)
        update_pose_velocity_filter(slow, 0.1, 0.05)
        self.assertNotAlmostEqual(filt.x2, slow.x2)

    def test_flatness_uses_world_velocity_pd(self) -> None:
        out = flatness_command(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.02
        )
        self.assertTrue(out.valid)
        self.assertGreater(out.linear_speed, 0.2)
        self.assertGreater(out.accel, 0.0)

    def test_flatness_rejects_fixed_period_trick_when_dt_invalid(self) -> None:
        out = flatness_command(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0
        )
        self.assertFalse(out.valid)

    def test_reset_plan_stays_inside_chassis_box(self) -> None:
        plan = plan_reset(0.0, 0.0, 0.0, 2.0, 0.0, 0.0)
        self.assertTrue(plan.valid)
        max_v, max_w, ok = max_box_on_plan(plan)
        self.assertTrue(ok)
        self.assertLessEqual(max_v, 1.05 + 1.0e-6)
        self.assertLessEqual(max_w, 1.05 + 1.0e-6)

    def test_reset_plan_succeeds_on_named_relative_poses(self) -> None:
        cases = (
            (1.2, -0.8, 0.7, "Q4"),
            (-0.5, 1.4, -2.8, "Q2-wrap"),
            (0.3, 0.3, 3.0, "near-pi"),
            (-1.5, -1.2, 0.0, "Q3-yaw0"),
            (1.0, 0.0, math.pi / 2.0, "east-90"),
            (0.0, 1.0, -math.pi / 2.0, "north-neg90"),
            (0.06, 0.0, 0.0, "just-outside-5cm"),
            (2.0, 1.5, -3.0, "far-wrap"),
            (-0.8, 0.8, 3.1, "Q2-pi"),
            (0.2, 0.0, math.pi, "facing-away-short"),
            (3.0, 0.0, math.pi, "facing-away-long"),
        )
        for x, y, yaw, name in cases:
            plan = plan_reset(x, y, yaw, 0.0, 0.0, 0.0)
            self.assertTrue(plan.valid, name)
            _, _, ok = max_box_on_plan(plan)
            self.assertTrue(ok, name)

    def test_reset_plan_random_relative_poses_all_succeed(self) -> None:
        rng = random.Random(20260904)
        failures = []
        for i in range(1000):
            dist = rng.uniform(0.06, 6.0)
            bearing = rng.uniform(-math.pi, math.pi)
            x = dist * math.cos(bearing)
            y = dist * math.sin(bearing)
            yaw = rng.uniform(-math.pi, math.pi)
            goal_yaw = rng.uniform(-math.pi, math.pi)
            plan = plan_reset(x, y, yaw, 0.0, 0.0, goal_yaw)
            if not plan.valid:
                failures.append((i, "invalid", x, y, yaw, goal_yaw, dist))
                continue
            max_v, max_w, ok = max_box_on_plan(plan)
            if not ok:
                failures.append((i, "box", x, y, yaw, goal_yaw, dist, max_v, max_w))
        self.assertEqual(failures, [], "planning failures: %s" % (failures[:12],))


if __name__ == "__main__":
    unittest.main()
