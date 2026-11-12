"""Servo configuration, command validation, and an explicitly simulated backend."""

import math
from dataclasses import dataclass, replace
from typing import Protocol


def _number(value, low, high, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be a finite number from {low} to {high}")


@dataclass(frozen=True)
class ServoConfig:
    port: str | None = None
    servo_id: int = 1
    baudrate: int = 115200
    timeout_s: float = 0.2
    write_timeout_s: float = 0.2
    min_angle_deg: float | None = None
    max_angle_deg: float | None = None
    max_step_deg: float = 5.0
    min_duration_s: float = 0.5
    max_duration_s: float = 2.0
    position_tolerance_deg: float = 2.0
    max_temperature_c: int = 60

    def __post_init__(self):
        if self.port is not None and (not isinstance(self.port, str) or not self.port.strip()):
            raise ValueError("port must be a nonempty device path or null")
        if type(self.servo_id) is not int or not 0 <= self.servo_id <= 253:
            raise ValueError("servo_id must be an integer from 0 to 253 (no broadcast)")
        if type(self.baudrate) is not int or self.baudrate != 115200:
            raise ValueError("LX-224 baudrate must be 115200")
        for name in ("timeout_s", "write_timeout_s"):
            _number(getattr(self, name), 0.01, 1.0, name)
        for name in ("min_angle_deg", "max_angle_deg"):
            if getattr(self, name) is not None:
                _number(getattr(self, name), 0, 240, name)
        if (self.min_angle_deg is None) != (self.max_angle_deg is None):
            raise ValueError("Set both mechanical angle limits, or leave both null for status")
        if self.min_angle_deg is not None and self.min_angle_deg >= self.max_angle_deg:
            raise ValueError("min_angle_deg must be less than max_angle_deg")
        _number(self.max_step_deg, 0.24, 240, "max_step_deg")
        _number(self.min_duration_s, 0.1, 30, "min_duration_s")
        _number(self.max_duration_s, self.min_duration_s, 30, "max_duration_s")
        _number(self.position_tolerance_deg, 0.24, 5, "position_tolerance_deg")
        if type(self.max_temperature_c) is not int or not 1 <= self.max_temperature_c <= 85:
            raise ValueError("max_temperature_c must be an integer from 1 to 85")

    def require_limits(self):
        if self.min_angle_deg is None:
            raise ValueError("Set measured min_angle_deg and max_angle_deg before motion")

    def validate_move(self, angle_deg, duration_s):
        """Check both requested and quantized targets before any I/O."""
        self.require_limits()
        _number(angle_deg, self.min_angle_deg, self.max_angle_deg, "angle_deg")
        _number(duration_s, self.min_duration_s, self.max_duration_s, "duration_s")
        position = round(angle_deg * 1000 / 240)
        milliseconds = round(duration_s * 1000)
        _number(position * 0.24, self.min_angle_deg, self.max_angle_deg, "quantized angle")
        _number(milliseconds / 1000, self.min_duration_s, self.max_duration_s, "quantized duration")
        return position, milliseconds


@dataclass(frozen=True)
class ServoStatus:
    position_deg: float
    voltage_v: float
    temperature_c: int
    mode: int
    torque_enabled: bool


def validate_feedback(status: ServoStatus, config: ServoConfig, *, require_torque=False):
    """Check servo health independently of joint calibration or torque release."""
    if status.mode != 0:
        raise ValueError("Servo is in motor mode; configure position mode before motion")
    _number(status.voltage_v, 6, 8.4, "servo voltage")
    _number(status.temperature_c, 0, config.max_temperature_c, "servo temperature")
    if require_torque and not status.torque_enabled:
        raise ValueError("Torque is disabled; explicitly enable it with 'torque on' first")


def validate_status(status: ServoStatus, config: ServoConfig, *, require_torque=False):
    validate_feedback(status, config, require_torque=require_torque)
    config.require_limits()
    _number(status.position_deg, config.min_angle_deg, config.max_angle_deg, "current position")


class ServoController(Protocol):
    def initialize(self) -> ServoStatus: ...
    def read_status(self) -> ServoStatus: ...
    def read_position_deg(self) -> float: ...
    def move_to(self, angle_deg: float, duration_s: float) -> None: ...
    def stop(self) -> None: ...
    def set_torque_enabled(self, enabled: bool) -> None: ...
    def read_id(self) -> int: ...
    def set_new_id(self, new_id: int) -> None: ...


class FakeServoController:
    """Records commands; position changes immediately, with no physical simulation."""

    def __init__(self, config: ServoConfig):
        self.config = config
        self.status = ServoStatus(120.0, 7.4, 25, 0, True)
        self.history = []
        self._opened = False
        self._ready = False
        self._active = False

    def open(self):
        if self._opened:
            raise RuntimeError("Servo is already open")
        self._opened = True
        self._ready = self._active = False
        return self

    def _require_open(self):
        if not self._opened:
            raise RuntimeError("Open the servo first (prefer a with block)")

    def read_status(self):
        self._require_open()
        return self.status

    def read_position_deg(self):
        return self.read_status().position_deg

    def read_voltage_v(self):
        return self.read_status().voltage_v

    def read_temperature_c(self):
        return self.read_status().temperature_c

    def read_id(self):
        self._require_open()
        return self.config.servo_id

    def set_new_id(self, new_id):
        self._require_open()
        config = replace(self.config, servo_id=new_id)
        if self._active:
            raise RuntimeError("Stop the servo before assigning an ID")
        if new_id != self.config.servo_id:
            self.history.append(("id", self.config.servo_id, new_id))
            self.config = config
            self._ready = False

    def initialize(self):
        self._ready = False
        status = self.read_status()
        validate_status(status, self.config)
        self._ready = True
        return status

    def move_to(self, angle_deg, duration_s):
        self._require_open()
        if not self._ready:
            raise RuntimeError("Initialize the servo before motion")
        position, milliseconds = self.config.validate_move(angle_deg, duration_s)
        validate_status(self.status, self.config, require_torque=True)
        target = position * 0.24
        _number(abs(target - self.status.position_deg), 0, self.config.max_step_deg, "step size")
        self._active = True
        self.history.append(("move", position, milliseconds))
        self.status = replace(self.status, position_deg=target)

    def stop(self):
        self._require_open()
        self.history.append(("stop",))
        self._active = False

    def set_torque_enabled(self, enabled):
        self._require_open()
        if type(enabled) is not bool:
            raise ValueError("enabled must be boolean")
        if enabled:
            self.initialize()
        self.stop()
        self.history.append(("torque", enabled))
        self.status = replace(self.status, torque_enabled=enabled)
        self._active = enabled

    def close(self):
        if self._opened:
            if self._active:
                self.stop()
            self._opened = self._ready = False

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        self.close()
        return False
