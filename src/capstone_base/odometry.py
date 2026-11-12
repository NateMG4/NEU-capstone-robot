"""Encoder-only X-layout mecanum odometry and an optional absolute-frame alignment."""

import math
from collections import deque
from dataclasses import dataclass

from .hiwonder_i2c import HiwonderConfig
from .kinematics import Geometry


def finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        for name in ("x", "y", "yaw"):
            finite(getattr(self, name), name)


def chassis_increment(distances, geometry):
    """Signed wheel rim distances FL, FR, RL, RR -> body dx, dy, CCW dyaw."""
    if len(distances) != 4:
        raise ValueError("Expected four wheel distances")
    fl, fr, rl, rr = (finite(d, "wheel distance") for d in distances)
    k = (geometry.wheelbase_m + geometry.track_width_m) / 2
    return ((fl + fr + rl + rr) / 4, (-fl + fr + rl - rr) / 4, (-fl + fr - rl + rr) / (4 * k))


def integrate(pose, dx, dy, yaw):
    """Exact SE(2) update assuming constant body twist over this increment."""
    half = yaw / 2
    scale = math.sin(half) / half if abs(half) > 1e-9 else 1.0
    c, s = math.cos(pose.yaw + half), math.sin(pose.yaw + half)
    return Pose(
        pose.x + scale * (c * dx - s * dy), pose.y + scale * (s * dx + c * dy), wrap(pose.yaw + yaw)
    )


def counter_delta(current, previous):
    """Small-step difference of two signed 32-bit counters, including rollover."""
    return ((current - previous + 2**31) % 2**32) - 2**31


def validate_counters(raw):
    if len(raw) != 4 or any(type(n) is not int or not -(2**31) <= n < 2**31 for n in raw):
        raise ValueError("Expected four signed int32 raw encoder counters")
    return tuple(raw)


class EncoderSampleError(ValueError):
    """A rejected sample plus evidence, retained without integrating the bad data."""

    def __init__(self, message, details):
        super().__init__(message)
        self.details = details


class EncoderOdometry:
    def __init__(self, geometry: Geometry, config: HiwonderConfig, *, max_wheel_rad_s=10.0):
        if any(
            value is None
            for value in (config.wheel_channels, config.encoder_signs, config.counts_per_wheel_rev)
        ):
            raise ValueError(
                "Odometry needs wheel channels, encoder signs and counts per revolution"
            )
        if finite(max_wheel_rad_s, "max_wheel_rad_s") <= 0:
            raise ValueError("max_wheel_rad_s must be positive")
        self.geometry, self.config = geometry, config
        self.max_wheel_rad_s = max_wheel_rad_s
        self.reset()

    def reset(self, pose=None):
        self.pose = Pose() if pose is None else pose
        self.raw = None
        self.time = None
        self.delta_counts = (0, 0, 0, 0)
        self.wheel_rad_s = (0.0, 0.0, 0.0, 0.0)
        self.wheel_distances = (0.0, 0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.history = deque(maxlen=400)

    def rebaseline(self, raw, now):
        """Discard uncertain intervening motion, preserving the last accepted pose/travel."""
        raw = validate_counters(raw)
        finite(now, "sample time")
        self.raw, self.time = raw, now
        self.delta_counts = (0, 0, 0, 0)
        self.wheel_rad_s = (0.0, 0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.history.clear()
        self.history.append((now, self.pose))

    def update(self, raw, now):
        finite(now, "sample time")
        raw = validate_counters(raw)
        if self.time is None:
            self.raw, self.time = raw, now
            self.history.append((now, self.pose))
            return self.pose
        dt = now - self.time
        if not 0 < dt <= 0.5:
            raise EncoderSampleError(
                f"Encoder sample gap or clock error: dt={dt:.6f}s; stop and rebaseline odometry",
                {"kind": "sample_timing", "dt_s": dt, "raw_previous": self.raw, "raw_current": raw},
            )
        deltas = tuple(
            counter_delta(raw[channel - 1], self.raw[channel - 1]) * sign
            for channel, sign in zip(
                self.config.wheel_channels, self.config.encoder_signs, strict=True
            )
        )
        # Two counts of quantization allowance. Resets that look like plausible motion
        # cannot be distinguished from real movement; known resets require reset().
        checks = []
        for name, channel, delta, scale in zip(
            ("FL", "FR", "RL", "RR"),
            self.config.wheel_channels,
            deltas,
            self.config.counts_per_wheel_rev,
            strict=True,
        ):
            allowed = self.max_wheel_rad_s * dt * scale / math.tau + 2
            inferred = delta / scale * math.tau / dt
            checks.append(
                {
                    "wheel": name,
                    "channel": channel,
                    "previous_count": self.raw[channel - 1],
                    "current_count": raw[channel - 1],
                    "delta_counts": delta,
                    "counts_per_wheel_rev": scale,
                    "allowed_delta_counts": allowed if math.isfinite(allowed) else None,
                    "inferred_rad_s": inferred if math.isfinite(inferred) else None,
                    "rejected": abs(delta) > allowed,
                }
            )
        rejected = [c for c in checks if c["rejected"]]
        if rejected:
            first = rejected[0]
            raise EncoderSampleError(
                f"Implausible encoder jump: {first['wheel']}/M{first['channel']} "
                f"delta={first['delta_counts']} counts in {dt * 1000:.1f}ms "
                f"(allowed {first['allowed_delta_counts']:.1f}); "
                "possible inconsistent read, counter reset, bad scale or timing",
                {
                    "kind": "encoder_jump",
                    "dt_s": dt,
                    "raw_previous": self.raw,
                    "raw_current": raw,
                    "max_wheel_rad_s": self.max_wheel_rad_s,
                    "wheels": checks,
                },
            )
        angles = tuple(
            n / scale * math.tau
            for n, scale in zip(deltas, self.config.counts_per_wheel_rev, strict=True)
        )
        distances = tuple(a * self.geometry.wheel_radius_m for a in angles)
        increment = chassis_increment(distances, self.geometry)
        pose = integrate(self.pose, *increment)
        self.pose, self.raw, self.time = pose, raw, now
        self.delta_counts = deltas
        self.wheel_rad_s = tuple(a / dt for a in angles)
        self.velocity = tuple(v / dt for v in increment)
        self.wheel_distances = tuple(
            a + b for a, b in zip(self.wheel_distances, distances, strict=True)
        )
        self.history.append((now, pose))
        return pose

    def pose_at(self, timestamp):
        """Interpolate bounded recent history for an observation's capture time."""
        finite(timestamp, "observation timestamp")
        if not self.history or not self.history[0][0] <= timestamp <= self.history[-1][0]:
            raise ValueError("Observation time is outside encoder history")
        previous_t, previous = self.history[0]
        for t, pose in self.history:
            if t == timestamp:
                return pose
            if t > timestamp:
                f = (timestamp - previous_t) / (t - previous_t)
                return Pose(
                    previous.x + f * (pose.x - previous.x),
                    previous.y + f * (pose.y - previous.y),
                    wrap(previous.yaw + f * wrap(pose.yaw - previous.yaw)),
                )
            previous_t, previous = t, pose
        raise ValueError("Observation time is outside encoder history")


class ReferenceFrame:
    """Rigid world<-odom alignment, separate from continuous encoder odometry.

    The optional camera adapter supplies a time-aligned absolute pose and the
    odom pose at capture time. This is an alignment hook, not an uncertainty filter.
    """

    def __init__(self):
        self.frame_id = "odom"
        self.offset = Pose()

    def transform(self, pose):
        c, s = math.cos(self.offset.yaw), math.sin(self.offset.yaw)
        return Pose(
            self.offset.x + c * pose.x - s * pose.y,
            self.offset.y + s * pose.x + c * pose.y,
            wrap(self.offset.yaw + pose.yaw),
        )

    def align(self, absolute_pose, odom_at_capture, frame_id):
        if not isinstance(frame_id, str) or not frame_id.strip() or frame_id == "odom":
            raise ValueError("Absolute observations need a named frame other than odom")
        yaw = wrap(absolute_pose.yaw - odom_at_capture.yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        self.offset = Pose(
            absolute_pose.x - c * odom_at_capture.x + s * odom_at_capture.y,
            absolute_pose.y - s * odom_at_capture.x - c * odom_at_capture.y,
            yaw,
        )
        self.frame_id = frame_id
