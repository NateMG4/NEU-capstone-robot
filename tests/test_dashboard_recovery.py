import json
import struct
import threading
import time

import pytest

from capstone_base.dashboard import DashboardRuntime, load_settings
from capstone_base.hiwonder_i2c import HiwonderController
from capstone_base.odometry import EncoderSampleError, Pose


class EncoderBoard:
    def __init__(self):
        self.raw = (0, 0, 0, 0)
        self.commands = []
        self.read_error = False
        self.noisy = False
        self.open_count = self.init_count = self.close_count = 0

    def open(self):
        self.open_count += 1

    def initialize(self):
        self.init_count += 1
        self.stop()

    def stop(self):
        self.commands.append((0, 0, 0, 0))

    def set_wheel_speeds(self, values):
        self.commands.append(values)

    def close(self):
        self.close_count += 1

    def read_encoder_counts(self):
        if self.read_error:
            raise OSError("Disconnected I2C")
        if self.noisy:
            self.raw = tuple(c + 100 for c in self.raw)
        return self.raw


@pytest.fixture
def live():
    board = EncoderBoard()
    runtime = DashboardRuntime(
        *load_settings("config/odometry-demo.json"), driver=board, allow_motion=True
    )
    board.open()
    board.initialize()
    runtime.hardware_open = True
    runtime.tick(10)
    runtime.command("mode", {"mode": "hardware"}, "tab", 10.001)
    return runtime, board


def reject_jump(runtime, board):
    board.raw = (10000, 0, 0, 0)
    with pytest.raises(EncoderSampleError) as caught:
        runtime.tick(10.05)
    runtime.trip(caught.value)


def test_recovery_is_queued_and_hardware_io_stays_outside_http_commands(live):
    runtime, board = live
    reject_jump(runtime, board)
    before = len(board.commands), board.init_count
    runtime.command("recover", {}, "tab", 10.1)
    assert (len(board.commands), board.init_count) == before
    assert runtime.recovery_requested
    assert runtime.fault
    with pytest.raises(ValueError, match="already"):
        runtime.command("recover", {}, "tab", 10.11)
    with pytest.raises(ValueError, match="latched"):
        runtime.command("apply", {}, "tab", 10.12)


def test_recovery_never_resumes_motion_and_keeps_the_last_valid_pose(live):
    runtime, board = live
    runtime.odometry["hardware"].pose = Pose(1, 2, 0.3)
    runtime.command("target", {"x": 1.5, "y": 2, "heading_deg": 20}, "tab", 10.002)
    runtime.command("apply", {}, "tab", 10.003)
    reject_jump(runtime, board)
    old_report = runtime.fault_report
    assert old_report["source"] == "hardware"
    assert old_report["latest_read"]["raw_counts"] == (10000, 0, 0, 0)
    runtime.command("recover", {}, "tab", 10.1)
    before = len(board.commands)
    runtime.recover(clock=lambda: 20, wait=lambda seconds: False)
    assert runtime.fault is None
    assert not runtime.active
    assert runtime.owner is None
    assert all(not any(c) for c in board.commands[before:])
    assert runtime.target == runtime.pose() == Pose(1, 2, 0.3)
    assert runtime.odometry["hardware"].raw == (10000, 0, 0, 0)
    assert runtime.pose_uncertain["hardware"]
    assert runtime.fault_report is old_report
    runtime.tick(20.05)
    runtime.command("heartbeat", {}, "tab", 20.06)
    assert not runtime.active
    assert runtime.snapshot()["recovery_count"] == 1
    json.dumps(runtime.snapshot(), allow_nan=False)
    with pytest.raises(ValueError, match="uncertain"):
        runtime.command("apply", {}, "tab", 20.07)
    runtime.command("reset", {}, "tab", 20.1)
    assert not runtime.pose_uncertain["hardware"]
    assert runtime.pose() == Pose()


@pytest.mark.parametrize("problem", ["noisy", "disconnected"])
def test_failed_recovery_keeps_fault_and_movement_disabled(live, problem):
    runtime, board = live
    reject_jump(runtime, board)
    board.noisy = problem == "noisy"
    board.read_error = problem == "disconnected"
    runtime.command("recover", {}, "tab", 10.1)
    runtime.recover(clock=lambda: 20, wait=lambda seconds: False)
    assert "Recovery failed" in runtime.fault
    assert not runtime.recovering
    assert not runtime.active
    assert not runtime.recovery_requested
    assert runtime.fault_report["details"]["kind"] == "encoder_jump"
    with pytest.raises(ValueError, match="latched"):
        runtime.command("apply", {}, "tab", 20.1)


def test_no_fault_cannot_queue_recovery(live):
    runtime, _ = live
    with pytest.raises(ValueError, match="no fault"):
        runtime.command("recover", {}, "tab", 10.1)


def test_jump_report_compares_counts_with_the_last_physical_motor_command(live):
    runtime, board = live
    runtime.command("target", {"x": 1, "y": 0, "heading_deg": 0}, "tab", 10.002)
    runtime.command("apply", {}, "tab", 10.003)
    runtime.tick(10.1)
    previous_command = runtime.hardware_sent
    board.raw = (10000, 0, 0, 0)
    with pytest.raises(EncoderSampleError) as caught:
        runtime.tick(10.15)
    runtime.trip(caught.value)
    report = runtime.fault_report
    assert report["requested_wheel_rad_s"] == previous_command
    assert report["motor_command_counts_per_10ms"] == (1, -1, -1, 1)
    assert report["expected_abs_delta_from_command"] == pytest.approx((5,) * 4)


def test_hardware_fault_while_simulating_does_not_report_virtual_commands(live):
    runtime, board = live
    runtime.command("mode", {"mode": "simulation"}, "tab", 10.002)
    runtime.command("target", {"x": 1, "y": 0, "heading_deg": 0}, "tab", 10.003)
    runtime.command("apply", {}, "tab", 10.004)
    runtime.tick(10.1)
    assert any(runtime.applied)
    board.raw = (10000, 0, 0, 0)
    with pytest.raises(EncoderSampleError) as caught:
        runtime.tick(10.15)
    runtime.trip(caught.value)
    assert runtime.fault_report["source"] == "hardware"
    assert runtime.fault_report["motor_command_counts_per_10ms"] == (0,) * 4
    assert runtime.fault_report["expected_abs_delta_from_command"] == (0,) * 4


def test_recovery_from_simulated_sample_fault_requires_no_hardware():
    runtime = DashboardRuntime(*load_settings("config/odometry-demo.json"))
    runtime.tick(10)
    runtime.trip(ValueError("sample fault"))
    runtime.command("recover", {}, "tab", 10.1)
    runtime.recover(clock=lambda: 20)
    runtime.tick(20.05)
    assert runtime.fault is None
    assert runtime.mode == "simulation"
    assert not runtime.active


def test_real_driver_communication_latch_is_reinitialized_with_zero_commands():
    class Bus:
        def __init__(self):
            self.fail = True
            self.writes = []

        def write_i2c_block_data(self, address, register, data):
            self.writes.append(data)

        def write_byte_data(self, address, register, value):
            pass

        def read_i2c_block_data(self, address, register, length):
            if self.fail:
                self.fail = False
                raise OSError("Transient read error")
            return list(struct.pack("<4i", 123, 456, 789, 234))

    geometry, config, limits = load_settings("config/odometry-demo.json")
    bus = Bus()
    driver = HiwonderController(config, bus=bus, sleep=lambda seconds: None)
    driver.open()
    driver.initialize()
    runtime = DashboardRuntime(geometry, config, limits, driver=driver, allow_motion=True)
    runtime.hardware_open = True
    with pytest.raises(OSError) as caught:
        runtime.tick(10)
    runtime.trip(caught.value)
    runtime.command("recover", {}, "tab", 10.1)
    runtime.recover(clock=lambda: 20, wait=lambda seconds: False)
    assert runtime.fault is None
    assert not runtime.active
    assert not driver._faulted
    assert all(data == [0, 0, 0, 0] for data in bus.writes)
    runtime.tick(20.05)


def test_worker_remains_available_after_initial_open_failure():
    class InitiallyDisconnected(EncoderBoard):
        def open(self):
            super().open()
            if self.open_count == 1:
                raise OSError("Not connected at startup")

    board = InitiallyDisconnected()
    runtime = DashboardRuntime(*load_settings("config/odometry-demo.json"), driver=board)
    thread = threading.Thread(target=runtime.run)
    thread.start()
    try:
        deadline = time.monotonic() + 2
        while not runtime.fault and time.monotonic() < deadline:
            time.sleep(0.01)
        assert runtime.fault
        runtime.command("recover", {}, "tab", time.monotonic())
        deadline = time.monotonic() + 2
        while runtime.fault and time.monotonic() < deadline:
            time.sleep(0.01)
        assert runtime.fault is None
        assert runtime.control_online
        assert not runtime.active
        assert board.open_count == 2
    finally:
        runtime.quit.set()
        thread.join(2)
    assert not thread.is_alive()
