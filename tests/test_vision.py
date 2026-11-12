import json
import math

import pytest

from capstone_base.dashboard import DashboardRuntime, load_settings
from capstone_base.odometry import Pose
from capstone_base.vision import MAX_PACKET_BYTES, VisionConfig, VisionReceiver


def packet(t=1000.05, frame=1, **robot):
    return json.dumps(
        {
            "t": t,
            "frame": frame,
            "robots": [{"id": 1, "x": 0.0, "y": 0.0, "heading": 0, "age": 0, **robot}],
            "user": {"x": 3, "y": 4, "confidence": 0.9},
        }
    ).encode()


def ingest(receiver, now=10.05, **robot):
    return receiver.ingest(
        packet(t=990 + now, frame=round(now * 100), **robot),
        ("127.0.0.1", 12345),
        now=now,
        wall_now=990 + now,
    )


@pytest.fixture
def runtime():
    receiver = VisionReceiver(VisionConfig())
    runtime = DashboardRuntime(*load_settings("config/odometry-demo.json"), vision=receiver)
    runtime.tick(10)
    return runtime


def align(runtime, **robot):
    assert ingest(runtime.vision, **robot)
    runtime.command("camera_align", {}, "tab", 10.06)
    runtime.tick(10.1)
    assert runtime.camera_enabled[runtime.mode]


def test_matches_partner_schema_and_uses_last_sighting_not_packet_time():
    receiver = VisionReceiver(VisionConfig(robot_id=2))
    assert ingest(receiver, id=2, age=0.04, x=1.42, y=3.1, heading=90, vx=0.0, vy=0.12)
    obs = receiver.observation(10.06)
    assert obs.pose == Pose(1.42, 3.1, math.pi / 2)
    assert obs.timestamp == pytest.approx(10.01)
    assert obs.epoch == pytest.approx(1000.01)
    assert receiver.snapshot(10.06)["age_s"] == pytest.approx(0.05)


@pytest.mark.parametrize(
    "robot",
    [
        {"id": 2},
        {"id": True},
        {"x": math.nan},
        {"y": math.inf},
        {"x": True},
        {"heading": None},
        {"heading": "90"},
        {"age": -0.1},
        {"age": 0.6},
    ],
)
def test_invalid_robot_data_never_replaces_last_pose(robot):
    receiver = VisionReceiver(VisionConfig())
    assert ingest(receiver)
    previous = receiver.latest
    assert not ingest(receiver, now=10.1, **robot)
    assert receiver.latest is previous
    assert receiver.rejected == 1


@pytest.mark.parametrize(
    "data",
    [
        b"[]",
        b"{",
        b"\xff",
        b"{}",
        b"x" * (MAX_PACKET_BYTES + 1),
        b'{"t":1000.05,"frame":true,"robots":[]}',
        b'{"t":1000.05,"frame":1,"robots":null}',
    ],
)
def test_malformed_datagrams_do_not_crash(data):
    receiver = VisionReceiver(VisionConfig())
    assert not receiver.ingest(data, ("127.0.0.1", 1), now=10.05, wall_now=1000.05)
    assert receiver.latest is None


def test_rejects_duplicate_ids_sources_stale_packets_clock_skew_and_order():
    receiver = VisionReceiver(VisionConfig())
    data = json.loads(packet())
    data["robots"] *= 2
    assert not receiver.ingest(json.dumps(data), ("127.0.0.1", 1), now=10.05, wall_now=1000.05)
    assert not receiver.ingest(packet(), ("192.168.50.99", 1), now=10.05, wall_now=1000.05)
    assert not receiver.ingest(packet(t=999), ("127.0.0.1", 1), now=10.05, wall_now=1000.05)
    assert not receiver.ingest(packet(t=1001), ("127.0.0.1", 1), now=10.05, wall_now=1000.05)
    assert ingest(receiver)
    assert not ingest(receiver)
    assert not ingest(receiver, now=10.04)
    # A publisher restart may reset frame numbers. Increasing epoch time is sufficient.
    assert receiver.ingest(packet(t=1000.1, frame=0), ("127.0.0.1", 1), now=10.1, wall_now=1000.1)


def test_age_includes_transport_delay_and_repeated_tracks_do_not_refresh_pose():
    receiver = VisionReceiver(VisionConfig())
    assert ingest(receiver)
    previous = receiver.latest
    assert ingest(receiver, now=10.25, age=0.2)
    assert receiver.latest is previous
    assert receiver.observation(10.6) is None
    assert not receiver.snapshot(10.6)["connected"]
    # Neither age nor transport delay alone exceeds the limit, but their sum does.
    assert not receiver.ingest(
        packet(t=1000.3, age=0.3),
        ("127.0.0.1", 1),
        now=10.6,
        wall_now=1000.6,
    )


@pytest.mark.parametrize(
    "settings",
    [
        {"port": 0},
        {"robot_id": True},
        {"listen_host": "not-an-ip"},
        {"source_ip": "not-an-ip"},
        {"frame_id": "odom"},
        {"frame_id": ""},
        {"max_age_s": 0.6},
        {"max_position_jump_m": -1},
        {"max_heading_jump_deg": 181},
    ],
)
def test_invalid_configuration_fails_before_network_start(settings):
    with pytest.raises(ValueError):
        VisionConfig(**settings)


def test_receiving_alone_never_aligns_or_grants_motion(runtime):
    assert ingest(runtime.vision, x=5)
    runtime.tick(10.1)
    assert runtime.pose() == Pose()
    assert not runtime.active
    assert runtime.frames["simulation"].frame_id == "odom"


def test_alignment_preserves_since_capture_motion_and_leaves_raw_odometry_untouched(runtime):
    assert ingest(runtime.vision, now=10.05, x=4, y=5, heading=90, age=0.05)
    runtime.command("camera_align", {}, "tab", 10.06)
    # Move all normalized wheels forwards after the camera's capture time (10.0).
    for channel, sign in zip(
        runtime.config.wheel_channels, runtime.config.encoder_signs, strict=True
    ):
        runtime.sim.positions[channel - 1] = sign * 100
    runtime.tick(10.1)
    odo = runtime.odometry["simulation"]
    assert odo.pose.x > 0
    assert runtime.pose().x == pytest.approx(4)
    assert runtime.pose().y == pytest.approx(5 + odo.pose.x)
    assert runtime.frames["simulation"].frame_id == "floor"
    assert runtime.target == runtime.pose()
    assert not runtime.active


def test_continuous_updates_preserve_target_and_encoder_fallback_keeps_same_frame(runtime):
    align(runtime)
    runtime.command("target", {"x": 0.5, "y": 0, "heading_deg": 0}, "tab", 10.11)
    runtime.command("apply", {}, "tab", 10.12)
    runtime.tick(10.15)
    assert ingest(runtime.vision, now=10.15, x=0.02)
    runtime.tick(10.2)
    assert runtime.active
    assert runtime.target == Pose(0.5, 0, 0)
    assert runtime.pose().x >= 0.02
    assert runtime.odometry["simulation"].pose.x < runtime.pose().x
    offset = runtime.frames["simulation"].offset
    for i in range(5, 17):
        now = 10 + i * 0.05
        runtime.command("heartbeat", {}, "tab", now)
        runtime.tick(now)
    assert runtime.active
    assert runtime.vision.observation(10.8) is None
    assert runtime.frames["simulation"].offset == offset
    assert runtime.frames["simulation"].frame_id == "floor"
    assert runtime.target == Pose(0.5, 0, 0)
    # Small, consistent reacquisition continues the same goal.
    pose = runtime.pose()
    assert ingest(runtime.vision, now=10.8, x=pose.x, y=pose.y)
    runtime.tick(10.85)
    assert runtime.active
    assert runtime.camera_error["simulation"] is None


@pytest.mark.parametrize("robot", [{"x": 1}, {"heading": 90}])
def test_large_discrepancy_stops_and_requires_explicit_realignment(runtime, robot):
    align(runtime)
    runtime.command("target", {"x": 0.5, "y": 0, "heading_deg": 0}, "tab", 10.11)
    runtime.command("apply", {}, "tab", 10.12)
    runtime.tick(10.15)
    runtime.tick(10.2)
    assert any(runtime.sim.speeds)
    assert ingest(runtime.vision, now=10.2, **robot)
    runtime.tick(10.25)
    assert not runtime.active
    assert not any(runtime.sim.speeds)
    assert not runtime.camera_enabled["simulation"]
    assert "discrepancy" in runtime.camera_error["simulation"]
    assert ingest(runtime.vision, now=10.3)
    runtime.tick(10.35)
    assert not runtime.active
    assert not runtime.camera_enabled["simulation"]


def test_repeated_corrections_do_not_prevent_arrival(runtime):
    align(runtime)
    runtime.command("apply", {}, "tab", 10.11)
    for i in range(3, 12):
        now = 10 + i * 0.05
        assert ingest(runtime.vision, now=now)
        runtime.tick(now)
    assert not runtime.active
    assert "Target reached" in runtime.status


def test_reset_disables_camera_and_cannot_align_across_new_encoder_history(runtime):
    align(runtime, x=4)
    runtime.command("reset", {}, "tab", 10.11)
    assert not runtime.camera_enabled["simulation"]
    runtime.tick(10.15)
    runtime.command("camera_align", {}, "tab", 10.16)
    runtime.tick(10.2)
    assert not runtime.camera_enabled["simulation"]
    assert "history" in runtime.camera_error["simulation"]
    assert runtime.pose() == Pose()
    assert runtime.fault is None
    assert ingest(runtime.vision, now=10.25, x=4)
    runtime.command("camera_align", {}, "tab", 10.26)
    runtime.tick(10.3)
    assert runtime.pose().x == 4


def test_fault_disables_camera_and_packets_cannot_clear_fault_or_resume_motion(runtime):
    align(runtime)
    runtime.trip(ValueError("bad encoder"))
    assert not runtime.camera_enabled["simulation"]
    assert ingest(runtime.vision, now=10.2, x=4)
    runtime.tick(10.25)
    with pytest.raises(ValueError, match="latched"):
        runtime.command("camera_align", {}, "tab", 10.26)
    assert runtime.pose_uncertain["simulation"]
    assert runtime.fault == "bad encoder"
    assert not runtime.active


def test_alignment_requires_fresh_pose_and_mode_change_cancels_pending_request(runtime):
    with pytest.raises(ValueError, match="fresh"):
        runtime.command("camera_align", {}, "tab", 10.01)
    assert ingest(runtime.vision)
    runtime.command("camera_align", {}, "tab", 10.06)
    runtime.command("mode", {"mode": "simulation"}, "tab", 10.07)
    runtime.tick(10.1)
    assert runtime.camera_align_requested is None
    assert not runtime.camera_enabled["simulation"]


def test_disable_keeps_frame_and_target_and_disarms(runtime):
    align(runtime, x=4)
    runtime.command("target", {"x": 4.5, "y": 0, "heading_deg": 0}, "tab", 10.11)
    runtime.command("apply", {}, "tab", 10.12)
    runtime.command("camera_disable", {}, "tab", 10.13)
    runtime.tick(10.15)
    assert runtime.pose().x == 4
    assert runtime.target.x == 4.5
    assert runtime.frames["simulation"].frame_id == "floor"
    assert not runtime.active
    assert not runtime.camera_enabled["simulation"]


def test_live_camera_alignment_and_discrepancy_stop_use_control_thread_only():
    class Board:
        def __init__(self):
            self.outputs = []

        def read_encoder_counts(self):
            return (0, 0, 0, 0)

        def set_wheel_speeds(self, wheels):
            self.outputs.append(wheels)

        def stop(self):
            self.outputs.append((0, 0, 0, 0))

    board = Board()
    runtime = DashboardRuntime(
        *load_settings("config/odometry-demo.json"),
        driver=board,
        allow_motion=True,
        vision=VisionReceiver(VisionConfig()),
    )
    runtime.tick(10)
    runtime.command("mode", {"mode": "hardware"}, "tab", 10.01)
    align(runtime, x=4)
    assert runtime.frames["simulation"].frame_id == "odom"
    assert not runtime.camera_enabled["simulation"]
    runtime.command("target", {"x": 4.5, "y": 0, "heading_deg": 0}, "tab", 10.11)
    runtime.command("apply", {}, "tab", 10.12)
    runtime.tick(10.15)
    assert any(board.outputs[-1])
    outputs = list(board.outputs)
    assert ingest(runtime.vision, now=10.15, x=6)
    assert board.outputs == outputs  # UDP receipt never performs motor I/O.
    runtime.tick(10.2)
    assert board.outputs[-1] == (0, 0, 0, 0)
    assert not runtime.active
    assert not runtime.camera_enabled["hardware"]
