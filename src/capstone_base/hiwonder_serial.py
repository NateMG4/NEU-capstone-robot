"""LX-224 protocol over BusLinker UART or USB serial; imports never open hardware.

One process owns the port. The lock serializes requests within that process.
Injected transports must be open, implement bounded I/O, and remain caller-owned.
"""

import os
import struct
import threading
import time
import warnings
from dataclasses import replace
from typing import Protocol

from .servo import ServoConfig, ServoStatus, _number, validate_status

MOVE = 1
STOP = 12
SERVO_ID_WRITE = 13
SERVO_ID_READ = 14
TEMPERATURE = 26
VOLTAGE = 27
POSITION = 28
MODE = 30
TORQUE_WRITE = 31
TORQUE_READ = 32


class SerialPort(Protocol):
    timeout: float
    write_timeout: float

    def write(self, data: bytes) -> int: ...
    def read(self, size: int) -> bytes: ...
    def reset_input_buffer(self) -> None: ...
    def close(self) -> None: ...


def encode_packet(servo_id: int, command: int, parameters: bytes = b"") -> bytes:
    if type(servo_id) is not int or not 0 <= servo_id <= 253:
        raise ValueError("Expected unicast servo ID 0..253")
    if type(command) is not int or not 0 <= command <= 255:
        raise ValueError("Expected command byte")
    if not isinstance(parameters, bytes) or len(parameters) > 4:
        raise ValueError("Expected at most four parameter bytes")
    body = bytes((servo_id, len(parameters) + 3, command)) + parameters
    return b"\x55\x55" + body + bytes((~sum(body) & 0xFF,))


class LX224Bus:
    """One serial connection and transaction lock for a chain of addressed servos.

    Enter the bus before its controllers; closing it cleans up every controller.
    A communication fault invalidates all controllers sharing the connection.
    """

    def __init__(
        self, config: ServoConfig, *, port: SerialPort | None = None, clock=time.monotonic
    ):
        self.config = config
        self._port = port
        self._owns_port = port is None
        self._clock = clock
        self._lock = threading.RLock()
        self._opened = False
        self._controllers = []

    def controller(self, config: ServoConfig):
        return LX224Controller(config, bus=self)

    def _register(self, controller):
        with self._lock:
            if (controller.config.port, controller.config.baudrate) != (
                self.config.port,
                self.config.baudrate,
            ):
                raise ValueError("All servos on a bus must use the same port and baudrate")
            if any(c.config.servo_id == controller.config.servo_id for c in self._controllers):
                raise ValueError(f"Duplicate servo ID {controller.config.servo_id} on this bus")
            self._controllers.append(controller)

    def open(self):
        with self._lock:
            if self._opened:
                raise RuntimeError("Servo bus is already open")
            if self._owns_port:
                if self.config.port is None:
                    raise ValueError("Set port to the BusLinker's actual serial device")
                try:
                    import serial
                except ImportError as error:
                    raise RuntimeError(
                        "Install servo dependencies: uv sync --extra servo "
                        "(or python -m pip install -e '.[servo]')"
                    ) from error
                self._port = serial.Serial(
                    port=self.config.port,
                    baudrate=self.config.baudrate,
                    bytesize=8,
                    parity="N",
                    stopbits=1,
                    timeout=self.config.timeout_s,
                    write_timeout=self.config.write_timeout_s,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                    exclusive=True if os.name == "posix" else None,
                )
            self._opened = True
            return self

    def _require_open(self):
        if not self._opened:
            raise RuntimeError("Open the servo bus first (prefer a with block)")

    def _send(self, config, command, parameters=b""):
        packet = encode_packet(config.servo_id, command, parameters)
        self._port.write_timeout = config.write_timeout_s
        if self._port.write(packet) != len(packet):
            raise OSError("Incomplete serial write; command delivery is uncertain")

    def _trip(self, error):
        for controller in self._controllers:
            if controller._opened:
                controller._trip(error)

    def _query(self, config, command, size, *, allow_missing=False):
        with self._lock:
            self._require_open()
            try:
                # Discard buffered stale replies before sending, never after sending.
                self._port.reset_input_buffer()
                self._send(config, command)
                deadline = self._clock() + config.timeout_s
                buffer = bytearray()
                while True:
                    remaining = deadline - self._clock()
                    if remaining <= 0:
                        if allow_missing:
                            return None
                        raise TimeoutError(
                            f"No valid reply from servo {config.servo_id}, command {command}; "
                            "check power, ID, USB/Servo jumpers, port, and 115200 baud"
                        )
                    self._port.timeout = remaining
                    buffer.extend(self._port.read(1))
                    # Bound storage even when the port supplies arbitrary leading noise.
                    while len(buffer) >= 2:
                        if buffer[:2] != b"\x55\x55":
                            del buffer[0]
                            continue
                        if len(buffer) < 4:
                            break
                        length = buffer[3]
                        if not 3 <= length <= 7:
                            del buffer[0]
                            continue
                        total = length + 3
                        if len(buffer) < total:
                            break
                        packet = bytes(buffer[:total])
                        if (sum(packet[2:]) & 0xFF) != 0xFF:
                            del buffer[0]
                            continue
                        del buffer[:total]
                        # Request echoes have zero parameters and are ignored here.
                        if (
                            packet[2] == config.servo_id
                            and packet[4] == command
                            and length == size + 3
                        ):
                            if self._clock() >= deadline:
                                raise TimeoutError("Servo reply arrived after the deadline")
                            return packet[5:-1]
            except OSError as error:
                self._trip(error)
                raise

    def close(self):
        with self._lock:
            if not self._opened:
                return
            try:
                first_error = None
                for controller in self._controllers:
                    try:
                        controller.close()
                    except OSError as error:
                        if first_error is None:
                            first_error = error
                        else:
                            warnings.warn(
                                f"Servo cleanup also failed: {error}", RuntimeWarning, stacklevel=2
                            )
                if first_error is not None:
                    raise first_error
            finally:
                self._opened = False
                if self._owns_port:
                    try:
                        self._port.close()
                    finally:
                        self._port = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except OSError as close_error:
            if exc is None:
                raise
            warnings.warn(
                f"Servo cleanup failed while handling {exc}: {close_error}",
                RuntimeWarning,
                stacklevel=2,
            )
        return False


class LX224Controller:
    """One servo, optionally on a shared LX224Bus with other controllers."""

    def __init__(
        self,
        config: ServoConfig,
        *,
        port: SerialPort | None = None,
        clock=time.monotonic,
        bus: LX224Bus | None = None,
    ):
        if bus is not None and port is not None:
            raise ValueError("Pass a shared bus or an injected port, not both")
        self.config = config
        self._owns_bus = bus is None
        self._bus = bus if bus is not None else LX224Bus(config, port=port, clock=clock)
        self._lock = self._bus._lock
        self._opened = self._ready = self._faulted = self._active = False
        self._bus._register(self)

    def open(self):
        with self._lock:
            if self._opened:
                raise RuntimeError("Servo is already open")
            if self._owns_bus:
                self._bus.open()
            else:
                self._bus._require_open()
            self._opened = True
            self._ready = self._faulted = self._active = False
            return self

    def _require_open(self):
        if not self._opened:
            raise RuntimeError("Open the servo first (prefer a with block)")
        self._bus._require_open()

    def _send(self, command, parameters=b""):
        self._bus._send(self.config, command, parameters)

    def _query(self, command, size):
        with self._lock:
            self._require_open()
            return self._bus._query(self.config, command, size)

    def _trip(self, error):
        self._ready = False
        self._faulted = True
        if self._active:
            self._active = False  # One bounded attempt; do not loop on a failed connection.
            try:
                self._send(STOP)
            except OSError as stop_error:
                warnings.warn(
                    f"Servo {self.config.servo_id} stop failed: {stop_error}; original: {error}",
                    RuntimeWarning,
                    stacklevel=2,
                )

    def read_position_deg(self):
        return struct.unpack("<h", self._query(POSITION, 2))[0] * 0.24

    def read_voltage_v(self):
        return struct.unpack("<H", self._query(VOLTAGE, 2))[0] / 1000

    def read_temperature_c(self):
        return self._query(TEMPERATURE, 1)[0]

    def read_status(self):
        with self._lock:
            mode = self._query(MODE, 4)[0]
            torque = self._query(TORQUE_READ, 1)[0]
            if mode not in (0, 1) or torque not in (0, 1):
                error = OSError("Invalid mode or torque value in servo response")
                self._bus._trip(error)
                raise error
            voltage = self.read_voltage_v()
            temperature = self.read_temperature_c()
            # Position last so the movement step check uses the freshest measurement.
            position = self.read_position_deg()
            return ServoStatus(position, voltage, temperature, mode, bool(torque))

    def read_id(self):
        with self._lock:
            servo_id = self._query(SERVO_ID_READ, 1)[0]
            if servo_id != self.config.servo_id:
                error = OSError("Servo ID readback does not match its packet address")
                self._bus._trip(error)
                raise error
            return servo_id

    def set_new_id(self, new_id):
        """Persist a unicast ID and verify it. Isolate servos with duplicate IDs first.

        Does not move, enable torque, or require travel calibration. After an
        uncertain write, config points at the new ID so callers can inspect it;
        the old or new ID may be in effect. No automatic write retry is made.
        """
        with self._lock:
            self._require_open()
            config = replace(self.config, servo_id=new_id)
            old_id = self.config.servo_id
            if any(c._active for c in self._bus._controllers):
                raise RuntimeError("Stop all servos on the bus before assigning an ID")
            if any(c is not self and c.config.servo_id == new_id for c in self._bus._controllers):
                raise ValueError(f"Servo ID {new_id} is already configured on this bus")
            self.read_id()
            if new_id == self.config.servo_id:
                return
            if self._bus._query(config, SERVO_ID_READ, 1, allow_missing=True) is not None:
                raise ValueError(f"Servo ID {new_id} is already responding on this bus")
            self._ready = False
            try:
                self._send(SERVO_ID_WRITE, bytes((new_id,)))
            except OSError as error:
                self.config = config
                self._bus._trip(error)
                raise OSError(
                    f"ID assignment uncertain; inspect IDs {old_id} and {new_id}: {error}"
                ) from error
            self.config = config
            try:
                self.read_id()
            except OSError as error:
                raise OSError(
                    f"ID assignment not verified; inspect IDs {old_id} and {new_id}: {error}"
                ) from error

    def initialize(self):
        with self._lock:
            self._ready = False
            try:
                status = self.read_status()
                validate_status(status, self.config)
            except (OSError, ValueError) as error:
                self._bus._trip(error)
                raise
            self._ready = True
            self._faulted = False
            return status

    def move_to(self, angle_deg, duration_s):
        with self._lock:
            self._require_open()
            if self._faulted or not self._ready:
                raise RuntimeError("Servo needs successful initialize() before motion")
            try:
                position, milliseconds = self.config.validate_move(angle_deg, duration_s)
                status = self.read_status()
                validate_status(status, self.config, require_torque=True)
                _number(
                    abs(position * 0.24 - status.position_deg),
                    0,
                    self.config.max_step_deg,
                    "step size",
                )
                self._active = True  # Even a partial write needs best-effort cleanup.
                self._send(MOVE, struct.pack("<HH", position, milliseconds))
            except (OSError, ValueError) as error:
                self._bus._trip(error)
                raise

    def stop(self):
        """Request a stop without changing torque; available even after a fault."""
        with self._lock:
            self._require_open()
            self._active = False
            try:
                self._send(STOP)
            except OSError:
                self._bus._trip("Stop write failed")
                raise

    def set_torque_enabled(self, enabled):
        with self._lock:
            self._require_open()
            if type(enabled) is not bool:
                raise ValueError("enabled must be boolean")
            if enabled:
                self.initialize()
            # Stop a previous target before changing torque state.
            self.stop()
            try:
                self._active = enabled
                self._send(TORQUE_WRITE, bytes((int(enabled),)))
                if self._query(TORQUE_READ, 1)[0] != int(enabled):
                    raise OSError("Servo torque readback did not match the requested state")
            except OSError as error:
                self._bus._trip(error)
                raise

    def close(self):
        with self._lock:
            if not self._opened:
                return
            try:
                if self._active:
                    self.stop()
            finally:
                self._opened = self._ready = False
                if self._owns_bus:
                    self._bus.close()

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except OSError as close_error:
            if exc is None:
                raise
            warnings.warn(
                f"Servo cleanup failed while handling {exc}: {close_error}",
                RuntimeWarning,
                stacklevel=2,
            )
        return False
