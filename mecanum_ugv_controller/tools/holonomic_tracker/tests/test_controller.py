#!/usr/bin/env python3
import math
import unittest

from holonomic_tracker.controller import (
    box_saturate,
    heading_rate_to_target,
    reset_command,
    track_command,
    world_velocity_to_body,
    wrap_angle,
)


class HolonomicTrackerTests(unittest.TestCase):
    def test_world_x_velocity_is_body_forward_when_yaw_is_zero(self) -> None:
        vx, vy = world_velocity_to_body(0.0, 1.0, 0.0)
        self.assertAlmostEqual(vx, 1.0)
        self.assertAlmostEqual(vy, 0.0)

    def test_world_x_velocity_is_body_right_when_facing_plus_y(self) -> None:
        vx, vy = world_velocity_to_body(math.pi / 2.0, 1.0, 0.0)
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, -1.0, places=6)

    def test_heading_rate_drives_yaw_to_world_x(self) -> None:
        rate = heading_rate_to_target(0.4, target_yaw=0.0, kp_yaw=1.2, max_yaw_rate=1.0)
        self.assertLess(rate, 0.0)
        self.assertAlmostEqual(heading_rate_to_target(0.0, target_yaw=0.0), 0.0)

    def test_track_command_keeps_heading_separate_from_body_velocity(self) -> None:
        output = track_command(0.2, 0.5, 0.0, target_yaw=0.0)
        self.assertGreater(output.linear_x, 0.0)
        self.assertLess(output.angular_z, 0.0)

    def test_box_saturate_clamps_each_axis(self) -> None:
        vx, vy, omega = box_saturate(2.0, -3.0, 4.0, max_linear_speed=1.0, max_yaw_rate=1.0)
        self.assertAlmostEqual(vx, 1.0)
        self.assertAlmostEqual(vy, -1.0)
        self.assertAlmostEqual(omega, 1.0)

    def test_track_command_uses_box_not_disk(self) -> None:
        output = track_command(0.0, 3.0, 3.0)
        self.assertAlmostEqual(output.linear_x, 1.0)
        self.assertAlmostEqual(output.linear_y, 1.0)

    def test_reset_arrives_within_five_cm(self) -> None:
        output = reset_command(0.02, -0.03, 0.4, 0.0, 0.0, 0.0)
        self.assertTrue(output.position_ok)

    def test_wrap_angle_is_principal_value(self) -> None:
        self.assertAlmostEqual(wrap_angle(math.pi + 0.1), -math.pi + 0.1, places=6)


if __name__ == "__main__":
    unittest.main()
