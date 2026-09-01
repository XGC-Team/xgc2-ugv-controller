"""Canonical holonomic tracking law.

The chassis tracker owns heading and the world-to-body rotation. Experiment
algorithms in Custom only publish a planar world-frame integrator velocity
(v_wx, v_wy). Converting with the measured yaw each tick avoids integrating
yaw flicker into a drifting body command.

C++ runtime must stay in lockstep with this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin


def wrap_angle(value: float) -> float:
    return atan2(sin(value), cos(value))


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def world_velocity_to_body(yaw: float, v_wx: float, v_wy: float) -> tuple[float, float]:
    """Rotate world ENU velocity into ROS FLU body: forward (x), left (y)."""
    c = cos(yaw)
    s = sin(yaw)
    return (c * v_wx + s * v_wy, -s * v_wx + c * v_wy)


def heading_rate_to_target(
    yaw: float,
    *,
    target_yaw: float = 0.0,
    kp_yaw: float = 1.2,
    max_yaw_rate: float = 0.6,
) -> float:
    """P control that holds the nose on world X (target_yaw=0) by default."""
    error = wrap_angle(target_yaw - yaw)
    return clamp(kp_yaw * error, -max_yaw_rate, max_yaw_rate)


@dataclass(frozen=True)
class TrackOutput:
    linear_x: float
    linear_y: float
    angular_z: float


def track_command(
    yaw: float,
    v_wx: float,
    v_wy: float,
    *,
    target_yaw: float = 0.0,
    kp_yaw: float = 1.2,
    max_speed: float = 0.8,
    max_yaw_rate: float = 0.6,
) -> TrackOutput:
    vx, vy = world_velocity_to_body(yaw, v_wx, v_wy)
    speed = (vx * vx + vy * vy) ** 0.5
    if speed > max_speed and speed > 0.0:
        scale = max_speed / speed
        vx *= scale
        vy *= scale
    return TrackOutput(
        linear_x=vx,
        linear_y=vy,
        angular_z=heading_rate_to_target(
            yaw, target_yaw=target_yaw, kp_yaw=kp_yaw, max_yaw_rate=max_yaw_rate
        ),
    )
