import struct
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from types import SimpleNamespace

import pytest

from capstone_base.hiwonder_serial import LX224Bus, LX224Controller, encode_packet
from capstone_base.servo import ServoConfig


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def reply(servo_id, command, payload):
    # Independent reference checksum, not the driver's encoder.
    data = bytes([servo_id, len(payload) + 3, command]) + payload
    return b"\x55\x55" + data + bytes([255 - sum(data) % 256])


class RecordingSerial:
    def __init__(self, clock=None):
        self.clock = clock or Clock()
        self.timeout = 0.2
        self.write_timeout = 0.2
        self.events = []
        self.rx = bytearray()
        self.closed = False
        self.position = 500
        self.mode = 0
        self.torque = 1
        self.voltage = 7400
        self.temperature = 25
        self.transform = lambda data, request: data
        self.fail_command = None
        self.short_command = None
        self.read_error = False
        self.noise_forever = False
        self.follow_move = True

    def reset_input_buffer(self):
        self.events.append("reset")
        self.rx.clear()

    def write(self, data):
        self.events.append(data)
        command = data[4]
        if command == self.fail_command:
            raise OSError("USB disconnected")
        if command == self.short_command:
            return len(data) - 1
        responses = {
            26: bytes([self.temperature]),
            27: struct.pack("<H", self.voltage),
            28: struct.pack("<h", self.position),
            30: bytes([self.mode, 0, 0, 0]),
            32: bytes([self.torque]),
        }
        if command in responses:
            self.rx.extend(self.transform(reply(data[2], command, responses[command]), data))
        elif command == 1 and self.follow_move:
            self.position = struct.unpack("<H", data[5:7])[0]
        elif command == 31:
            self.torque = data[5]
        return len(data)

    def read(self, size):
        if self.read_error:
            raise OSError("read disconnected")
        self.clock.now += min(0.001, self.timeout)
        if self.noise_forever:
            return b"\x00"
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def close(self):
        self.closed = True

    @property
    def packets(self):
        return [event for event in self.events if isinstance(event, bytes)]


class RecordingChainSerial(RecordingSerial):
    """Independent addressed nodes on one wire, including persistent ID writes."""

    def __init__(self, ids=(1, 2)):
        super().__init__()
        self.nodes = {
            servo_id: dict(position=500, mode=0, torque=1, voltage=7400, temperature=25)
            for servo_id in ids
        }
        self.ignore_id_write = False
        self.close_count = 0

    def write(self, data):
        servo_id, command = data[2], data[4]
        if command in (self.fail_command, self.short_command):
            return super().write(data)
        if servo_id not in self.nodes:
            self.events.append(data)
            return len(data)
        if command == 14:
            self.events.append(data)
            self.rx.extend(self.transform(reply(servo_id, 14, bytes([servo_id])), data))
        elif command == 13:
            self.events.append(data)
            if not self.ignore_id_write:
                self.nodes[data[5]] = self.nodes.pop(servo_id)
        else:
            state = self.nodes[servo_id]
            for name, value in state.items():
                setattr(self, name, value)
            result = super().write(data)
            state.update({name: getattr(self, name) for name in state})
            return result
        return len(data)

    def close(self):
        self.close_count += 1
        super().close()


@pytest.fixture
def setup():
    config = ServoConfig(port="/dev/test-servo", min_angle_deg=115, max_angle_deg=125)
    port = RecordingSerial()
    driver = LX224Controller(config, port=port, clock=port.clock)
    return driver, port


def test_known_packets():
    assert encode_packet(1, 28) == bytes.fromhex("55 55 01 03 1c df")
    assert encode_packet(1, 12) == bytes.fromhex("55 55 01 03 0c ef")
    assert encode_packet(1, 1, struct.pack("<HH", 500, 1000)) == bytes.fromhex(
        "55 55 01 07 01 f4 01 e8 03 16"
    )


@pytest.mark.parametrize("servo_id", (1, 2))
def test_standalone_driver_needs_only_the_addressed_servo(servo_id):
    port = RecordingChainSerial(ids=(servo_id,))
    config = ServoConfig(servo_id=servo_id, min_angle_deg=115, max_angle_deg=125)
    with LX224Controller(config, port=port, clock=port.clock) as driver:
        assert driver.initialize().position_deg == 120
        driver.move_to(122, 1)
        assert driver.read_position_deg() == pytest.approx(121.92)
        driver.stop()
        driver.set_torque_enabled(False)
        assert not driver.read_status().torque_enabled
        driver.set_torque_enabled(True)
        assert driver.read_status().torque_enabled
    assert all(packet[2] == servo_id for packet in port.packets)
    assert not any(packet[4] == 13 for packet in port.packets)


def test_inert_construction_read_only_status_and_injected_ownership(setup):
    driver, port = setup
    assert port.events == []
    with driver:
        assert port.events == []
        status = driver.read_status()
        assert status.position_deg == 120
        assert status.voltage_v == 7.4
        assert status.temperature_c == 25
        assert status.mode == 0
        assert status.torque_enabled
    assert [p[4] for p in port.packets] == [30, 32, 27, 26, 28]
    assert not port.closed


def test_signed_position_and_stale_buffer_discard(setup):
    driver, port = setup
    port.position = -5
    port.rx.extend(reply(1, 28, struct.pack("<h", 999)))
    with driver:
        assert driver.read_position_deg() == -1.2


def test_noise_wrong_id_wrong_command_echo_checksum_and_fragmented_reply(setup):
    driver, port = setup

    def transform(data, request):
        corrupt = data[:-1] + bytes([data[-1] ^ 1])
        return (
            b"\x00\x55\x55\x01\xff"
            + reply(2, 28, b"\x00\x00")
            + reply(1, 27, b"\x00\x00")
            + request
            + corrupt
            + data
        )

    port.transform = transform
    with driver:
        assert driver.read_position_deg() == 120
    assert len(port.packets) == 1  # No request retries.


@pytest.mark.parametrize(
    "transform",
    [
        lambda data, request: b"",
        lambda data, request: data[:-1],
        lambda data, request: data[:-1] + bytes([data[-1] ^ 1]),
        lambda data, request: request,
        lambda data, request: reply(2, 28, b"\x00\x00"),
        lambda data, request: reply(1, 28, b"\x00"),
    ],
)
def test_missing_or_invalid_reply_times_out_without_writes_to_actuators(setup, transform):
    driver, port = setup
    port.transform = transform
    with driver, pytest.raises(TimeoutError):
        driver.read_position_deg()
    assert port.clock.now <= 0.201
    assert [p[4] for p in port.packets] == [28]


def test_noise_does_not_extend_deadline(setup):
    driver, port = setup
    port.noise_forever = True
    with driver, pytest.raises(TimeoutError):
        driver.read_position_deg()
    assert port.clock.now <= 0.201


def test_move_exact_bytes_cleanup_and_fresh_step_limit(setup):
    driver, port = setup
    with driver:
        driver.initialize()
        port.position = 499  # Changed since initialization.
        driver.move_to(122, 1)
    assert port.packets[-2] == bytes.fromhex("55 55 01 07 01 fc 01 e8 03 0e")
    assert port.packets[-1] == bytes.fromhex("55 55 01 03 0c ef")
    assert not any(p[4] == 31 for p in port.packets)


def test_step_limit_uses_latest_position(setup):
    driver, port = setup
    with driver:
        driver.initialize()
        port.position = 481  # 115.44 degrees; target 122 is now too far away.
        with pytest.raises(ValueError, match="step size"):
            driver.move_to(122, 1)
    assert not any(p[4] in (1, 12, 31) for p in port.packets)


@pytest.mark.parametrize(
    "attribute,value",
    [("mode", 1), ("voltage", 12000), ("temperature", 61), ("torque", 0)],
)
def test_motion_checks_fresh_health_and_torque(setup, attribute, value):
    driver, port = setup
    with driver:
        driver.initialize()
        setattr(port, attribute, value)
        with pytest.raises(ValueError):
            driver.move_to(122, 1)
    assert not any(p[4] in (1, 31) for p in port.packets)


def test_partial_motion_write_stops_latches_fault_and_can_reinitialize(setup):
    driver, port = setup
    with driver:
        driver.initialize()
        port.short_command = 1
        with pytest.raises(OSError, match="Incomplete"):
            driver.move_to(122, 1)
        assert port.packets[-1][4] == 12
        count = len(port.packets)
        with pytest.raises(RuntimeError, match="initialize"):
            driver.move_to(122, 1)
        assert len(port.packets) == count
        port.short_command = None
        driver.initialize()
        driver.move_to(122, 1)
    assert [p[4] for p in port.packets].count(1) == 2


def test_read_fault_during_active_motion_stops_once(setup):
    driver, port = setup
    with driver:
        driver.initialize()
        driver.move_to(122, 1)
        port.read_error = True
        with pytest.raises(OSError, match="read disconnected"):
            driver.read_position_deg()
    assert [p[4] for p in port.packets].count(12) == 1


def test_explicit_torque_and_release_without_calibration(setup):
    driver, port = setup
    port.torque = 0
    with driver:
        driver.set_torque_enabled(True)
        assert driver.read_status().torque_enabled
    assert port.packets[-1][4] == 12
    assert port.torque == 1  # Closing holds; it does not release the joint.
    driver = LX224Controller(ServoConfig(), port=port, clock=port.clock)
    with driver:
        driver.set_torque_enabled(False)
    assert port.torque == 0
    assert [p[4] for p in port.packets[-3:]] == [12, 31, 32]


def test_owned_port_settings_and_close(monkeypatch):
    port = RecordingSerial()
    settings = {}

    def factory(**kwargs):
        settings.update(kwargs)
        return port

    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=factory))
    with LX224Controller(ServoConfig(port="/dev/example"), clock=port.clock) as driver:
        driver.read_status()
    assert port.closed
    assert settings["baudrate"] == 115200
    assert (settings["bytesize"], settings["parity"], settings["stopbits"]) == (8, "N", 1)
    assert settings["timeout"] == settings["write_timeout"] == 0.2
    assert not settings["rtscts"] and not settings["xonxoff"]


def test_owned_port_closed_when_stop_fails_and_original_exception_preserved(monkeypatch, setup):
    driver, port = setup
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=lambda **kw: port))
    owned = LX224Controller(driver.config, clock=port.clock)
    with (
        pytest.warns(RuntimeWarning, match="cleanup failed"),
        pytest.raises(ValueError, match="first"),
    ):
        with owned:
            owned.initialize()
            owned.move_to(122, 1)
            port.fail_command = 12
            raise ValueError("first")
    assert port.closed


def test_missing_dependency_is_actionable(monkeypatch, setup):
    driver, _ = setup
    monkeypatch.setitem(sys.modules, "serial", None)
    with pytest.raises(RuntimeError, match="servo dependencies"):
        LX224Controller(driver.config).open()


def test_read_only_initialization_needs_limits(setup):
    _, port = setup
    with LX224Controller(ServoConfig(), port=port, clock=port.clock) as driver:
        with pytest.raises(ValueError, match="measured"):
            driver.initialize()
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets)


def test_invalid_target_rejected_before_any_io(setup):
    driver, port = setup
    with driver:
        driver.initialize()
        port.events.clear()
        with pytest.raises(ValueError):
            driver.move_to(float("nan"), 1)
        assert not port.events


def test_reopen_never_replays_motion(setup):
    driver, port = setup
    with driver:
        driver.initialize()
        driver.move_to(122, 1)
    port.events.clear()
    with driver:
        assert port.events == []
        with pytest.raises(RuntimeError):
            driver.move_to(122, 1)
    assert port.events == []


def test_read_status_rejects_invalid_mode_and_torque_values(setup):
    driver, port = setup
    port.mode = 255
    with driver, pytest.raises(OSError, match="Invalid mode"):
        driver.read_status()


def test_quantized_step_limit(setup):
    driver, port = setup
    driver.config = replace(driver.config, max_step_deg=0.25)
    with driver:
        driver.initialize()
        with pytest.raises(ValueError, match="step size"):
            driver.move_to(120.36, 1)  # Rounds to 120.48, beyond the limit.


@pytest.mark.parametrize("new_id", [0, 2, 253])
def test_assign_id_without_limits_verifies_new_address_and_preserves_torque(new_id):
    port = RecordingChainSerial(ids=(1,))
    config = ServoConfig()
    with LX224Controller(config, port=port, clock=port.clock) as driver:
        driver.set_new_id(new_id)
        assert driver.config.servo_id == driver.read_id() == new_id
        assert driver.read_status().torque_enabled
        with pytest.raises(RuntimeError, match="initialize"):
            driver.move_to(122, 1)
    assert config.servo_id == 1  # Caller can persist config explicitly.
    assert list(port.nodes) == [new_id]
    assert [p[4] for p in port.packets[:4]] == [14, 14, 13, 14]
    assert port.packets[2] == reply(1, 13, bytes([new_id]))
    assert not any(p[4] in (1, 12, 31) for p in port.packets)
    assert not port.closed


def test_known_id_packets():
    assert encode_packet(1, 13, b"\x02") == bytes.fromhex("55 55 01 04 0d 02 eb")
    assert encode_packet(2, 14) == bytes.fromhex("55 55 02 03 0e ec")


@pytest.mark.parametrize("new_id", [-1, 254, 255, True, 2.0, None])
def test_bad_new_id_rejected_before_io(new_id):
    port = RecordingChainSerial()
    with LX224Controller(ServoConfig(), port=port, clock=port.clock) as driver:
        with pytest.raises(ValueError):
            driver.set_new_id(new_id)
    assert port.events == []


def test_same_id_reads_without_persistent_write():
    port = RecordingChainSerial()
    with LX224Controller(ServoConfig(), port=port, clock=port.clock) as driver:
        driver.set_new_id(1)
    assert [p[4] for p in port.packets] == [14]


def test_id_conflict_on_wire_rejected_without_write():
    port = RecordingChainSerial()
    with LX224Controller(ServoConfig(), port=port, clock=port.clock) as driver:
        with pytest.raises(ValueError, match="already responding"):
            driver.set_new_id(2)
        assert driver.config.servo_id == 1
    assert [p[4] for p in port.packets] == [14, 14]
    assert set(port.nodes) == {1, 2}


@pytest.mark.parametrize("failure", ["missing_source", "short_write", "ignored_write", "bad_id"])
def test_id_assignment_failure_is_not_retried_and_latches_motion(failure):
    port = RecordingChainSerial(ids=() if failure == "missing_source" else (1,))
    if failure == "short_write":
        port.short_command = 13
    elif failure == "ignored_write":
        port.ignore_id_write = True
    elif failure == "bad_id":
        port.transform = lambda data, request: reply(request[2], 14, b"\x03")
    with LX224Controller(ServoConfig(), port=port, clock=port.clock) as driver:
        with pytest.raises(OSError):
            driver.set_new_id(2)
        assert driver._faulted
        with pytest.raises(RuntimeError, match="initialize"):
            driver.move_to(122, 1)
    assert sum(p[4] == 13 for p in port.packets) <= 1
    assert all(p[4] in (13, 14) for p in port.packets)


def test_assignment_requires_inactive_bus_and_invalidates_initialization():
    config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
    port = RecordingChainSerial(ids=(1,))
    with LX224Controller(config, port=port, clock=port.clock) as driver:
        driver.initialize()
        driver.move_to(122, 1)
        count = len(port.packets)
        with pytest.raises(RuntimeError, match="Stop all"):
            driver.set_new_id(3)
        assert len(port.packets) == count
        driver.stop()
        driver.set_new_id(3)
        with pytest.raises(RuntimeError, match="initialize"):
            driver.move_to(120, 1)
        driver.initialize()
        driver.move_to(120, 1)
    assert port.packets[-1][2:5] == bytes([3, 3, 12])


def test_shared_bus_routes_independent_feedback_moves_and_cleanup():
    config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
    port = RecordingChainSerial()
    port.nodes[2]["position"] = 495
    bus = LX224Bus(config, port=port, clock=port.clock)
    first = bus.controller(config)
    second = bus.controller(replace(config, servo_id=2, max_step_deg=3))
    with bus:
        first.open()
        second.open()
        first.initialize()
        second.initialize()
        first.move_to(122, 1)
        second.move_to(120, 1)
        assert first.read_position_deg() == pytest.approx(121.92)
        assert second.read_position_deg() == 120
        assert port.nodes[1]["position"] == 508
        assert port.nodes[2]["position"] == 500
        first.close()
        assert second.read_position_deg() == 120
    assert [(p[2], p[4]) for p in port.packets if p[4] in (1, 12)] == [
        (1, 1),
        (2, 1),
        (1, 12),
        (2, 12),
    ]
    assert not port.closed
    with pytest.raises(RuntimeError, match="Open"):
        second.read_status()


def test_shared_bus_rejects_duplicate_ids_ports_and_conflicting_assignment():
    config = ServoConfig()
    port = RecordingChainSerial()
    with LX224Bus(config, port=port, clock=port.clock) as bus:
        with bus.controller(config) as first:
            bus.controller(replace(config, servo_id=2))
            with pytest.raises(ValueError, match="Duplicate"):
                bus.controller(config)
            with pytest.raises(ValueError, match="same port"):
                bus.controller(replace(config, servo_id=3, port="/dev/other"))
            with pytest.raises(ValueError, match="already configured"):
                first.set_new_id(2)
    assert port.events == []


def test_shared_bus_requires_open_and_reopen_never_resumes():
    config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
    port = RecordingChainSerial()
    bus = LX224Bus(config, port=port, clock=port.clock)
    driver = bus.controller(config)
    with pytest.raises(RuntimeError, match="Open the servo bus"):
        driver.open()
    with bus, driver:
        driver.initialize()
        driver.move_to(122, 1)
    port.events.clear()
    with bus, driver:
        with pytest.raises(RuntimeError, match="initialize"):
            driver.move_to(122, 1)
    assert port.events == []


def test_chain_communication_fault_stops_every_active_servo_and_blocks_motion():
    config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
    port = RecordingChainSerial()
    with LX224Bus(config, port=port, clock=port.clock) as bus:
        with bus.controller(config) as first, bus.controller(replace(config, servo_id=2)) as second:
            for driver in (first, second):
                driver.initialize()
                driver.move_to(122, 1)
            port.nodes.pop(2)  # Only the second servo stops replying.
            with pytest.raises(TimeoutError):
                second.read_position_deg()
            assert [(p[2], p[4]) for p in port.packets[-2:]] == [(1, 12), (2, 12)]
            for driver in (first, second):
                with pytest.raises(RuntimeError, match="initialize"):
                    driver.move_to(120, 1)
    assert sum(p[4] == 12 for p in port.packets) == 2


def test_owned_bus_opens_once_and_closes_once_after_failed_cleanup(monkeypatch):
    config = ServoConfig(port="/dev/test", min_angle_deg=115, max_angle_deg=125)
    port = RecordingChainSerial()
    opens = []

    def factory(**kwargs):
        opens.append(kwargs)
        return port

    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=factory))
    with pytest.raises(OSError, match="USB disconnected"):
        with LX224Bus(config, clock=port.clock) as bus:
            for servo_id in (1, 2):
                driver = bus.controller(replace(config, servo_id=servo_id)).open()
                driver.initialize()
                driver.move_to(122, 1)
            port.fail_command = 12
            # Both stops will fail; preserve the first error and report the other.
            with pytest.warns(RuntimeWarning, match="stop failed"):
                bus.close()
    assert len(opens) == port.close_count == 1
    assert all(not c._opened for c in bus._controllers)
    assert {(p[2], p[4]) for p in port.packets if p[4] == 12} == {(1, 12), (2, 12)}


def test_shared_bus_serializes_concurrent_request_reply_transactions(monkeypatch):
    config = ServoConfig()
    port = RecordingChainSerial()
    port.nodes[2]["position"] = 400
    read = port.read

    def yielding_read(size):
        time.sleep(0)  # Allow the other thread to run during fragmented responses.
        return read(size)

    monkeypatch.setattr(port, "read", yielding_read)
    barrier = threading.Barrier(2)

    def positions(driver):
        barrier.wait(timeout=2)
        return [driver.read_position_deg() for _ in range(10)]

    with LX224Bus(config, port=port, clock=port.clock) as bus:
        with bus.controller(config) as first, bus.controller(replace(config, servo_id=2)) as second:
            with ThreadPoolExecutor(max_workers=2) as pool:
                first_result = pool.submit(positions, first)
                second_result = pool.submit(positions, second)
                assert first_result.result(timeout=5) == [120] * 10
                assert second_result.result(timeout=5) == [96] * 10
    assert len(port.packets) == 20
