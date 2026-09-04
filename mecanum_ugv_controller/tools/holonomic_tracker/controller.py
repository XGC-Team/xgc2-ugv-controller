"""Reusable holonomic first-order and Reset laws.

C++ runtime must stay in lockstep with this module. Not a paper algorithm.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, sin


def wrap_angle(value: float) -> float:
    return atan2(sin(value), cos(value))


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def world_velocity_to_body(yaw: float, v_wx: float, v_wy: float) -> tuple[float, float]:
    """Rotate world ENU velocity into ROS FLU body: forward (x), left (y)."""
    c = cos(yaw)
    s = sin(yaw)
    return (c * v_wx + s * v_wy, -s * v_wx + c * v_wy)


def box_saturate(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    *,
    max_linear_speed: float = 1.0,
    max_yaw_rate: float = 1.0,
) -> tuple[float, float, float]:
    """Clamp FLU vx, vy, ω. Not world-frame velocity."""
    return (
        clamp(linear_x, -max_linear_speed, max_linear_speed),
        clamp(linear_y, -max_linear_speed, max_linear_speed),
        clamp(angular_z, -max_yaw_rate, max_yaw_rate),
    )


def heading_rate_to_target(
    yaw: float,
    *,
    target_yaw: float = 0.0,
    kp_yaw: float = 1.2,
    max_yaw_rate: float = 1.0,
) -> float:
    error = wrap_angle(target_yaw - yaw)
    return clamp(kp_yaw * error, -max_yaw_rate, max_yaw_rate)


@dataclass(frozen=True)
class TrackOutput:
    linear_x: float
    linear_y: float
    angular_z: float


@dataclass(frozen=True)
class ResetOutput:
    linear_x: float
    linear_y: float
    angular_z: float
    position_ok: bool


def track_command(
    yaw: float,
    v_wx: float,
    v_wy: float,
    *,
    target_yaw: float = 0.0,
    kp_yaw: float = 1.2,
    max_linear_speed: float = 1.0,
    max_yaw_rate: float = 1.0,
) -> TrackOutput:
    vx, vy = world_velocity_to_body(yaw, v_wx, v_wy)
    omega = heading_rate_to_target(
        yaw, target_yaw=target_yaw, kp_yaw=kp_yaw, max_yaw_rate=max_yaw_rate
    )
    vx, vy, omega = box_saturate(
        vx, vy, omega, max_linear_speed=max_linear_speed, max_yaw_rate=max_yaw_rate
    )
    return TrackOutput(linear_x=vx, linear_y=vy, angular_z=omega)


def reset_command(
    x: float,
    y: float,
    yaw: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    *,
    kp_xy: float = 0.8,
    kp_yaw: float = 1.2,
    max_linear_speed: float = 1.0,
    max_yaw_rate: float = 1.0,
    arrive_position: float = 0.05,
) -> ResetOutput:
    ex = goal_x - x
    ey = goal_y - y
    vx, vy = world_velocity_to_body(yaw, kp_xy * ex, kp_xy * ey)
    omega = kp_yaw * wrap_angle(goal_yaw - yaw)
    vx, vy, omega = box_saturate(
        vx, vy, omega, max_linear_speed=max_linear_speed, max_yaw_rate=max_yaw_rate
    )
    return ResetOutput(
        linear_x=vx,
        linear_y=vy,
        angular_z=omega,
        position_ok=hypot(ex, ey) <= arrive_position,
    )
