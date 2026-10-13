"""X-layout mecanum kinematics; SI units and FL, FR, RL, RR wheel order."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Geometry:
    wheel_radius_m: float
    wheelbase_m: float
    track_width_m: float

    def __post_init__(self):
        for value in (self.wheel_radius_m, self.wheelbase_m, self.track_width_m):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Geometry dimensions must be finite and positive")


def wheel_speeds(
    vx: float, vy: float, wz: float, geometry: Geometry, max_wheel_rad_s: float
) -> tuple[float, float, float, float]:
    """Forward vx, left vy, counterclockwise wz -> wheel rad/s.

    Apply motor wiring signs only in the hardware adapter. Saturation
    scales all wheels together to preserve the requested velocity ratios.
    """
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        raise ValueError("Velocity commands must be finite")
    if not math.isfinite(max_wheel_rad_s) or max_wheel_rad_s <= 0:
        raise ValueError("Wheel speed limit must be finite and positive")
    k = geometry.wheelbase_m / 2 + geometry.track_width_m / 2
    r = geometry.wheel_radius_m
    wheels = (
        (vx - vy - k * wz) / r,
        (vx + vy + k * wz) / r,
        (vx + vy - k * wz) / r,
        (vx - vy + k * wz) / r,
    )
    if not all(math.isfinite(value) for value in wheels):
        raise ValueError("Calculated wheel speed overflow")
    peak = max(abs(value) for value in wheels)
    scale = max_wheel_rad_s / peak if peak > max_wheel_rad_s else 1.0
    return tuple(value * scale for value in wheels)
