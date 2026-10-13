"""Keyboard-to-motor mixing and a local command lease; no terminal or I2C I/O."""

import math
from typing import Protocol

from .hiwonder_i2c import HiwonderConfig

DIRECTIONS = {
    "w": (1, 0, 0),
    "s": (-1, 0, 0),
    "a": (0, 1, 0),
    "d": (0, -1, 0),
    "q": (0, 0, 1),
    "e": (0, 0, -1),
    "7": (1, 1, 0),
    "9": (1, -1, 0),
    "1": (-1, 1, 0),
    "3": (-1, -1, 0),
}
ZERO = (0, 0, 0, 0)


class MotorOutput(Protocol):
    def set_motor_speed_counts(self, values: tuple[int, ...]) -> None: ...
    def stop(self) -> None: ...


class FakeMotorOutput:
    """Record raw M1..M4 counts/10ms, without importing/opening hardware."""

    def __init__(self):
        self.values = ZERO
        self.history = []

    def set_motor_speed_counts(self, values):
        self.values = tuple(values)
        self.history.append(self.values)

    def stop(self):
        self.set_motor_speed_counts(ZERO)


def mix_direction(axes, speed):
    """Dimensionless forward/left/CCW input -> FL, FR, RL, RR counts/10ms.

    This bring-up mixer assumes matching wheels/encoder scales and an X roller
    layout. It is not a calibrated m/s or rad/s chassis velocity interface.
    """
    if len(axes) != 3 or not all(math.isfinite(v) and abs(v) <= 1 for v in axes):
        raise ValueError("Expected three finite direction components in -1..1")
    if not math.isfinite(speed) or speed < 0:
        raise ValueError("Speed must be finite and nonnegative")
    x, y, yaw = axes
    raw = (x - y - yaw, x + y + yaw, x + y - yaw, x - y + yaw)
    scale = speed / max(1, *(abs(v) for v in raw))
    return tuple(v * scale for v in raw)


class TeleopState:
    def __init__(self, config: HiwonderConfig, *, speed=5, timeout=0.3, acceleration=20.0, now=0.0):
        if config.wheel_channels is None or config.motor_signs is None:
            raise ValueError("Teleoperation needs wheel_channels and motor_signs")
        if type(speed) is not int or not 1 <= speed <= config.max_speed_counts:
            raise ValueError("Speed must be an integer from 1 to max_speed_counts")
        if not math.isfinite(timeout) or not 0.1 <= timeout <= 1:
            raise ValueError("Input timeout must be between 0.1 and 1 second")
        if not math.isfinite(acceleration) or acceleration <= 0:
            raise ValueError("Acceleration must be finite and positive")
        if not math.isfinite(now):
            raise ValueError("Clock must be finite")
        self.config = config
        self.speed = speed
        self.timeout = timeout
        self.acceleration = acceleration
        self.last_tick = now
        self.last_input = None
        self.axes = (0, 0, 0)
        self.wheels = (0.0, 0.0, 0.0, 0.0)

    def stop(self):
        self.axes = (0, 0, 0)
        self.last_input = None
        self.wheels = (0.0, 0.0, 0.0, 0.0)

    def accept(self, key, now):
        if key in DIRECTIONS:
            self.axes = DIRECTIONS[key]
            self.last_input = now
        else:
            self.stop()

    def step(self, now):
        if not math.isfinite(now) or now < self.last_tick:
            self.stop()
            raise ValueError("Monotonic clock moved backwards or became nonfinite")
        dt = min(now - self.last_tick, 0.1)
        self.last_tick = now
        if self.last_input is None or now - self.last_input >= self.timeout:
            self.stop()  # Stop immediately on expiry, without ramping down.
            return ZERO
        target = mix_direction(self.axes, self.speed)
        differences = tuple(t - current for t, current in zip(target, self.wheels, strict=True))
        peak = max(abs(v) for v in differences)
        scale = min(1.0, self.acceleration * dt / peak) if peak else 1.0
        self.wheels = tuple(
            current + delta * scale for current, delta in zip(self.wheels, differences, strict=True)
        )
        motor = [0, 0, 0, 0]
        for value, channel, sign in zip(
            self.wheels, self.config.wheel_channels, self.config.motor_signs, strict=True
        ):
            motor[channel - 1] = round(value) * sign
        return tuple(motor)


class KeyDecoder:
    """Decode split ANSI arrow sequences without treating suffixes as WASD."""

    def __init__(self):
        self.escape = ""

    def feed(self, data):
        events = []
        for byte in data:
            char = chr(byte)
            if self.escape:
                self.escape += char
                if len(self.escape) == 2:
                    if char not in "[O":
                        self.escape = ""
                        events.append("stop")
                elif "@" <= char <= "~" or len(self.escape) > 32:
                    events.append(
                        {
                            "\x1b[A": "w",
                            "\x1b[B": "s",
                            "\x1b[C": "d",
                            "\x1b[D": "a",
                            "\x1bOA": "w",
                            "\x1bOB": "s",
                            "\x1bOC": "d",
                            "\x1bOD": "a",
                        }.get(self.escape, "stop")
                    )
                    self.escape = ""
            elif byte == 27:
                self.escape = char
            elif char.lower() in DIRECTIONS:
                events.append(char.lower())
            elif char.lower() == "c" or byte in (3, 4):
                events.append("quit")
            else:
                events.append("stop")
        return events
