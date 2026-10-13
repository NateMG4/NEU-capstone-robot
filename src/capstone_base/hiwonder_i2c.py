"""Hiwonder four-channel encoder motor module (expected board v1.3).

Registers follow the vendor TankDemo.py; see docs/hiwonder.md. All wheel-facing
values use FL, FR, RL, RR order. Raw methods use M1, M2, M3, M4 order.
Importing this module never imports smbus2 or opens hardware.
"""

import math
import struct
import time
import warnings
from dataclasses import dataclass
from typing import Protocol

from .controller import WheelTargets

MOTOR_TYPE = 0x14
ENCODER_POLARITY = 0x15
FIXED_SPEED = 0x33
ENCODER_TOTAL = 0x3C
BATTERY = 0x00


class I2CBus(Protocol):
    def write_byte_data(self, address: int, register: int, value: int) -> None: ...
    def write_i2c_block_data(self, address: int, register: int, data: list[int]) -> None: ...
    def read_i2c_block_data(self, address: int, register: int, length: int) -> list[int]: ...
    def close(self) -> None: ...


def _integer(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer from {low} to {high}")


def _four(values, name):
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError(f"{name} must contain four values in FL, FR, RL, RR order")
    return tuple(values)


@dataclass(frozen=True)
class HiwonderConfig:
    motor_type: int
    encoder_polarity: int
    wheel_channels: tuple[int, int, int, int] | None = None
    motor_signs: tuple[int, int, int, int] | None = None
    encoder_signs: tuple[int, int, int, int] | None = None
    counts_per_wheel_rev: tuple[float, float, float, float] | None = None
    max_speed_counts: int = 10
    bus_number: int = 1
    address: int = 0x34

    def __post_init__(self):
        _integer(self.motor_type, 1, 3, "motor_type (closed-loop encoder motors)")
        _integer(self.encoder_polarity, 0, 1, "encoder_polarity")
        _integer(self.max_speed_counts, 1, 127, "max_speed_counts")
        _integer(self.bus_number, 0, 255, "bus_number")
        _integer(self.address, 0x08, 0x77, "7-bit I2C address")
        for name in ("wheel_channels", "motor_signs", "encoder_signs", "counts_per_wheel_rev"):
            if getattr(self, name) is not None:
                object.__setattr__(self, name, _four(getattr(self, name), name))
        if self.wheel_channels is not None and (
            any(type(v) is not int for v in self.wheel_channels)
            or set(self.wheel_channels) != {1, 2, 3, 4}
        ):
            raise ValueError("wheel_channels must be a permutation of 1, 2, 3, 4")
        for name in ("motor_signs", "encoder_signs"):
            if getattr(self, name) is not None and any(
                type(v) is not int or v not in (-1, 1) for v in getattr(self, name)
            ):
                raise ValueError(f"{name} must contain only -1 or +1")
        if self.counts_per_wheel_rev is not None and any(
            type(v) not in (int, float) or not math.isfinite(v) or v <= 0
            for v in self.counts_per_wheel_rev
        ):
            raise ValueError("counts_per_wheel_rev must contain positive finite measured scales")


def encode_speed_counts(values, limit: int) -> list[int]:
    """Validate before converting signed commands to unsigned transport bytes."""
    _integer(limit, 1, 127, "speed limit")
    values = _four(values, "motor speeds")
    for value in values:
        _integer(value, -limit, limit, "motor speed in counts/10ms")
    return [value & 0xFF for value in values]


def wheel_targets_to_counts(targets: WheelTargets, config: HiwonderConfig) -> tuple[int, ...]:
    if any(
        value is None
        for value in (config.wheel_channels, config.motor_signs, config.counts_per_wheel_rev)
    ):
        raise ValueError("Wheel calibration is required for rad/s commands")
    targets = _four(targets, "wheel speeds")
    result = [0, 0, 0, 0]
    for speed, channel, sign, counts in zip(
        targets, config.wheel_channels, config.motor_signs, config.counts_per_wheel_rev, strict=True
    ):
        if type(speed) not in (int, float) or not math.isfinite(speed):
            raise ValueError("Wheel speeds must be finite numbers in rad/s")
        value = speed / math.tau * counts * 0.01 * sign
        if not math.isfinite(value) or abs(value) > config.max_speed_counts:
            raise ValueError("Requested speed exceeds max_speed_counts; reduce the wheel target")
        result[channel - 1] = round(value)
    return tuple(result)


class HiwonderController:
    """Explicit open/initialize lifecycle, closed-loop speed writes and telemetry.

    No automatic heartbeat/watchdog. The caller must enforce command expiry.
    Injected buses are caller-owned; SMBus instances opened here are closed here.
    Use from one controlling thread/process only.
    """

    def __init__(self, config: HiwonderConfig, *, bus: I2CBus | None = None, sleep=time.sleep):
        self.config = config
        self._bus = bus
        self._owns_bus = bus is None
        self._sleep = sleep
        self._opened = False
        self._initialized = False
        self._faulted = False

    def open(self):
        if self._opened:
            raise RuntimeError("Controller is already open")
        if self._owns_bus:
            try:
                from smbus2 import SMBus
            except ImportError as error:
                raise RuntimeError(
                    "Install dependencies: python -m pip install -e '.[pi]' (or uv sync --extra pi)"
                ) from error
            self._bus = SMBus(self.config.bus_number)
        self._opened = True
        self._initialized = False
        self._faulted = False
        return self

    def _require_open(self):
        if not self._opened:
            raise RuntimeError("Open the controller first (prefer a with block)")

    def _require_ready(self):
        self._require_open()
        if self._faulted or not self._initialized:
            raise RuntimeError("Controller needs successful initialize() before motion")

    def _zero(self):
        self._bus.write_i2c_block_data(self.config.address, FIXED_SPEED, [0, 0, 0, 0])

    def _trip(self, error):
        """Latch a fault, try zero once, and retain the original exception."""
        self._faulted = True
        self._initialized = False
        try:
            self._zero()
        except OSError as stop_error:
            # Do not claim the stop succeeded when the bus is unavailable.
            warnings.warn(
                f"Motor stop could not be confirmed: {stop_error}; original: {error}",
                RuntimeWarning,
                stacklevel=2,
            )

    def initialize(self):
        """Stop, set motor profile and global polarity, settle, and stop again."""
        self._require_open()
        self._initialized = False
        try:
            self._zero()
            self._bus.write_byte_data(self.config.address, MOTOR_TYPE, self.config.motor_type)
            self._sleep(0.5)  # Delay used in the vendor's Raspberry Pi example.
            self._bus.write_byte_data(
                self.config.address, ENCODER_POLARITY, self.config.encoder_polarity
            )
            self._sleep(0.5)
            self._zero()
        except BaseException as error:
            self._trip(error)
            raise
        self._initialized = True
        self._faulted = False

    def set_motor_speed_counts(self, values):
        """Raw M1..M4 signed integer commands in encoder counts per 10 ms."""
        self._require_ready()
        try:
            payload = encode_speed_counts(values, self.config.max_speed_counts)
            self._bus.write_i2c_block_data(self.config.address, FIXED_SPEED, payload)
        except (ValueError, TypeError, OSError) as error:
            self._trip(error)
            raise

    def set_wheel_speeds(self, targets: WheelTargets) -> None:
        self._require_ready()
        try:
            counts = wheel_targets_to_counts(targets, self.config)
        except (ValueError, TypeError, OverflowError) as error:
            self._trip(error)
            raise
        self.set_motor_speed_counts(counts)

    def stop(self) -> None:
        """Write zero even before initialization or after a fault."""
        self._require_open()
        try:
            self._zero()
        except OSError:
            self._faulted = True
            self._initialized = False
            raise

    def _read(self, register, size):
        self._require_open()
        try:
            data = self._bus.read_i2c_block_data(self.config.address, register, size)
            if (
                not isinstance(data, (list, tuple, bytes, bytearray))
                or len(data) != size
                or any(type(v) is not int or not 0 <= v <= 255 for v in data)
            ):
                raise OSError(f"Malformed I2C response at register 0x{register:02x}")
            return bytes(data)
        except OSError as error:
            self._trip(error)
            raise

    def read_encoder_counts(self) -> tuple[int, int, int, int]:
        """Raw signed 32-bit counts in M1..M4 order; counters can wrap/reset."""
        return struct.unpack("<4i", self._read(ENCODER_TOTAL, 16))

    def read_wheel_encoder_counts(self) -> tuple[int, ...]:
        if self.config.wheel_channels is None or self.config.encoder_signs is None:
            raise RuntimeError("Wheel calibration is required for wheel-ordered encoder counts")
        raw = self.read_encoder_counts()
        return tuple(
            raw[channel - 1] * sign
            for channel, sign in zip(
                self.config.wheel_channels, self.config.encoder_signs, strict=True
            )
        )

    def read_battery_voltage(self) -> float:
        """Board supply voltage in volts (wire value is unsigned little-endian mV)."""
        return struct.unpack("<H", self._read(BATTERY, 2))[0] / 1000.0

    def close(self):
        if not self._opened:
            return
        try:
            self.stop()
        finally:
            self._opened = False
            self._initialized = False
            if self._owns_bus:
                self._bus.close()
                self._bus = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except OSError as close_error:
            if exc is None:
                raise
            warnings.warn(
                f"Cleanup failed while handling {exc}: {close_error}", RuntimeWarning, stacklevel=2
            )
        return False
