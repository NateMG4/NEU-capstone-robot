"""Bounded pose feedback for an unobstructed local target; no hardware I/O."""

import math
from dataclasses import dataclass

from .kinematics import wheel_speeds
from .odometry import Pose, chassis_increment, finite, wrap


@dataclass(frozen=True)
class NavigationLimits:
    max_speed_m_s: float = 0.12
    max_yaw_rad_s: float = 0.4
    max_wheel_rad_s: float = 4.0
    wheel_accel_rad_s2: float = 6.0
    position_tolerance_m: float = 0.03
    yaw_tolerance_rad: float = math.radians(4)
    position_gain: float = 0.8
    yaw_gain: float = 1.2

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if finite(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")


def target_wheels(pose, target, geometry, limits):
    """Room error -> bounded body velocities -> ratio-preserving wheel targets."""
    ex, ey = target.x - pose.x, target.y - pose.y
    distance, yaw_error = math.hypot(ex, ey), wrap(target.yaw - pose.yaw)
    reached = distance <= limits.position_tolerance_m and abs(yaw_error) <= limits.yaw_tolerance_rad
    speed = min(limits.max_speed_m_s, limits.position_gain * distance)
    if distance <= limits.position_tolerance_m:
        vx = vy = 0.0
    else:
        vx, vy = ex * speed / distance, ey * speed / distance
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    wz = max(-limits.max_yaw_rad_s, min(limits.max_yaw_rad_s, limits.yaw_gain * yaw_error))
    if abs(yaw_error) <= limits.yaw_tolerance_rad:
        wz = 0.0
    wheels = wheel_speeds(c * vx + s * vy, -s * vx + c * vy, wz, geometry, limits.max_wheel_rad_s)
    body = chassis_increment(tuple(w * geometry.wheel_radius_m for w in wheels), geometry)
    return {
        "wheels": wheels,
        "body": body,
        "distance": distance,
        "yaw_error": yaw_error,
        "reached": reached,
    }


def ramp_wheels(current, target, dt, acceleration):
    dt = finite(dt, "dt")
    if not 0 < dt <= 0.5:
        raise ValueError("Invalid control interval")
    differences = tuple(b - a for a, b in zip(current, target, strict=True))
    peak = max(abs(d) for d in differences)
    scale = min(1.0, acceleration * dt / peak) if peak else 1.0
    return tuple(a + scale * d for a, d in zip(current, differences, strict=True))


def pose_dict(pose: Pose):
    return {"x": pose.x, "y": pose.y, "yaw": pose.yaw, "heading_deg": math.degrees(pose.yaw)}
