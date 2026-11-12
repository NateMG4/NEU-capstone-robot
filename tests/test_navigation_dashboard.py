import json
import math
from pathlib import Path

import pytest

from capstone_base import dashboard
from capstone_base.dashboard import DashboardRuntime, load_settings
from capstone_base.navigation import NavigationLimits, ramp_wheels, target_wheels
from capstone_base.odometry import Pose


@pytest.fixture
def settings():
    return load_settings("config/odometry-demo.json")


@pytest.fixture
def runtime(settings):
    runtime = DashboardRuntime(*settings)
    runtime.tick(10)
    return runtime


class MotorBoard:
    def __init__(self):
        self.commands = []

    def read_encoder_counts(self):
        return (0, 0, 0, 0)

    def set_wheel_speeds(self, targets):
        self.commands.append(targets)

    def stop(self):
        self.commands.append((0, 0, 0, 0))


def set_target(runtime, now=10.001, client="tab", **kwargs):
    runtime.command("target", {"x": 0.5, "y": 0.3, "heading_deg": 45, **kwargs}, client, now)


def test_world_target_requires_right_strafe_at_90_degrees(settings):
    geometry, _, limits = settings
    plan = target_wheels(Pose(0, 0, math.pi / 2), Pose(1, 0, math.pi / 2), geometry, limits)
    assert plan["body"][0] == pytest.approx(0, abs=1e-12)
    assert plan["body"][1] < 0
    assert plan["wheels"][0] > 0 and plan["wheels"][1] < 0


def test_diagonal_speed_limit_heading_wrap_and_arrival(settings):
    geometry, _, limits = settings
    plan = target_wheels(Pose(), Pose(1, 1, 0), geometry, limits)
    assert math.hypot(*plan["body"][:2]) <= limits.max_speed_m_s
    plan = target_wheels(
        Pose(0, 0, math.radians(179)), Pose(0, 0, math.radians(-175)), geometry, limits
    )
    assert 0 < plan["body"][2] <= limits.max_yaw_rad_s
    assert target_wheels(Pose(), Pose(0.01, 0.01, 0.01), geometry, limits)["reached"]


def test_wheel_acceleration_preserves_direction():
    assert ramp_wheels((0,) * 4, (2, -4, 1, -2), 0.1, 5) == (0.25, -0.5, 0.125, -0.25)
    with pytest.raises(ValueError):
        NavigationLimits(max_speed_m_s=math.nan)


def test_preview_never_moves_and_applied_simulation_arrives(runtime):
    set_target(runtime)
    runtime.tick(10.05)
    assert any(runtime.snapshot()["plan"]["wheels"])
    assert runtime.pose() == Pose()
    assert not any(runtime.snapshot()["applied_wheels"])
    runtime.command("apply", {}, "tab", 10.06)
    for i in range(2, 500):
        now = 10 + 0.05 * i
        runtime.command("heartbeat", {}, "tab", now)
        runtime.tick(now)
        if not runtime.active:
            break
    assert "reached" in runtime.status
    assert runtime.snapshot()["plan"]["distance"] <= runtime.limits.position_tolerance_m
    assert not any(runtime.sim.speeds)


def test_expired_heartbeat_latches_stop_without_auto_resume(runtime):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    for i in range(1, 18):
        runtime.tick(10 + i * 0.05)
    assert not runtime.active
    assert "expired" in runtime.status
    assert not any(runtime.applied)
    runtime.command("heartbeat", {}, "tab", 10.9)
    assert not runtime.active


def test_other_tab_cannot_renew_or_replace_active_target_but_can_stop(runtime):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    with pytest.raises(ValueError, match="Another"):
        runtime.command("heartbeat", {}, "other", 10.5)
    with pytest.raises(ValueError):
        set_target(runtime, now=10.5, client="other")
    assert runtime.lease_until == pytest.approx(10.76)
    runtime.command("stop", {}, "other", 10.6)
    assert not runtime.active


def test_target_change_mode_change_and_reset_disarm(runtime):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    set_target(runtime, now=10.02, x=0.2)
    assert not runtime.active
    runtime.command("apply", {}, "tab", 10.03)
    runtime.command("mode", {"mode": "simulation"}, "tab", 10.04)
    assert not runtime.active
    runtime.command("reset", {}, "tab", 10.05)
    assert runtime.odometry["simulation"].raw is None
    assert runtime.target == Pose()
    with pytest.raises(ValueError, match="baseline"):
        runtime.command("apply", {}, "tab", 10.06)


@pytest.mark.parametrize("bad", [math.nan, math.inf, True, "0.4", None])
def test_bad_targets_do_not_refresh_lease(runtime, bad):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    with pytest.raises(ValueError):
        set_target(runtime, now=10.5, x=bad)
    assert runtime.lease_until == pytest.approx(10.76)
    assert runtime.target.x == 0.5


def test_target_distance_bound_and_hardware_not_available(runtime):
    with pytest.raises(ValueError, match="2 m"):
        set_target(runtime, x=3)
    with pytest.raises(ValueError, match="Live encoders"):
        runtime.command("mode", {"mode": "hardware"}, "tab", 10.1)


def test_hardware_requires_launch_permission_and_expiry_stops_motor(settings):
    board = MotorBoard()
    runtime = DashboardRuntime(*settings, driver=board)
    runtime.tick(10)
    runtime.command("mode", {"mode": "hardware"}, "tab", 10.001)
    set_target(runtime, now=10.002)
    with pytest.raises(ValueError, match="disabled"):
        runtime.command("apply", {}, "tab", 10.003)
    assert board.commands == []
    runtime.allow_motion = True
    runtime.command("apply", {}, "tab", 10.004)
    runtime.tick(10.05)
    assert any(board.commands[-1])
    for i in range(2, 18):
        runtime.tick(10 + 0.05 * i)
    assert board.commands[-1] == (0, 0, 0, 0)
    assert not runtime.active


def test_simulation_never_drives_connected_hardware(settings):
    board = MotorBoard()
    runtime = DashboardRuntime(*settings, driver=board, allow_motion=True)
    runtime.tick(10)
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    runtime.tick(10.05)
    runtime.tick(10.10)
    runtime.tick(10.15)
    assert board.commands == []
    assert any(runtime.sim.speeds)


def test_fault_stops_and_cannot_be_rearmed(runtime):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    runtime.trip(OSError("I2C failed"))
    assert not runtime.active
    assert not any(runtime.sim.speeds)
    with pytest.raises(ValueError, match="latched"):
        runtime.command("apply", {}, "tab", 10.1)
    runtime.command("stop", {}, "any", 10.2)
    assert runtime.fault == "I2C failed"


def test_slow_read_cannot_extend_movement_lease(settings):
    board = MotorBoard()
    runtime = DashboardRuntime(*settings, driver=board, allow_motion=True)
    runtime.tick(10)
    runtime.command("mode", {"mode": "hardware"}, "tab", 10.001)
    set_target(runtime, now=10.002)
    runtime.command("apply", {}, "tab", 10.003)
    runtime.tick(10.35)
    assert any(board.commands[-1])
    runtime.tick(10.4, clock=lambda: 10.8)
    assert not runtime.active
    assert board.commands[-1] == (0, 0, 0, 0)


def test_repeated_apply_does_not_extend_move_limit(runtime):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    with pytest.raises(ValueError, match="already active"):
        runtime.command("apply", {}, "tab", 10.5)
    assert runtime.started == 10.01
    runtime.started = -20
    runtime.tick(10.1)
    assert not runtime.active
    assert "30-second" in runtime.status


def test_camera_seam_aligns_capture_time_and_clears_target(runtime):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    runtime.tick(10.05)
    runtime.tick(10.10)
    runtime.observe_pose(Pose(4, 5, 0), 10.05, "room", 10.11)
    assert runtime.frames["simulation"].frame_id == "room"
    assert not runtime.active
    assert runtime.pose().x >= 4
    assert runtime.target == runtime.pose()
    with pytest.raises(ValueError, match="stale"):
        runtime.observe_pose(Pose(), 9, "room", 10.11)


def test_hardware_rejects_demo_missing_calibration_and_respects_count_limit(settings, tmp_path):
    data = json.loads(Path("config/odometry-demo.json").read_text())
    data["simulation_only"] = True
    path = tmp_path / "robot.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="measured"):
        load_settings(path, hardware=True)
    data["simulation_only"] = False
    data["hiwonder"]["encoder_signs"] = None
    path = tmp_path / "robot.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="encoder scales"):
        load_settings(path, hardware=True)
    data["hiwonder"]["encoder_signs"] = [1, -1, -1, 1]
    data["hiwonder"]["max_speed_counts"] = 1
    path.write_text(json.dumps(data))
    settings = load_settings(path, hardware=True)
    assert settings[2].max_wheel_rad_s == pytest.approx(math.tau / 10)
    limited = DashboardRuntime(*settings)
    limited.tick(10)
    set_target(limited)
    assert max(abs(v) for v in limited.snapshot()["planned_motor_counts"]) <= 1


@pytest.mark.parametrize(
    ("module", "backend_flag"),
    [(dashboard, "--backend")],
)
def test_shipped_demo_cannot_open_hardware(module, backend_flag, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail("Demo settings must be rejected before opening hardware or a socket")

    monkeypatch.setattr(module, "HiwonderController", forbidden)
    monkeypatch.setattr(module, "make_server", forbidden)
    with pytest.raises(SystemExit) as error:
        module.main([backend_flag, "hiwonder"])
    assert error.value.code == 2
    assert "measured simulation_only=false" in capsys.readouterr().err


def test_invalid_cli_config_never_opens_hardware(monkeypatch, tmp_path):
    def forbidden(*args, **kwargs):
        pytest.fail("Invalid configuration must not open hardware or a socket")

    monkeypatch.setattr(dashboard, "HiwonderController", forbidden)
    monkeypatch.setattr(dashboard, "make_server", forbidden)
    path = tmp_path / "simulation.json"
    data = json.loads(Path("config/odometry-demo.json").read_text())
    data["simulation_only"] = True
    path.write_text(json.dumps(data))
    with pytest.raises(SystemExit) as error:
        dashboard.main(["--backend", "hiwonder", "--config", str(path)])
    assert error.value.code == 2


def test_navigation_updates_preview_and_preserves_target(runtime):
    set_target(runtime, y=0, heading_deg=0)
    before = runtime.snapshot()["plan"]["body"][0]
    runtime.command(
        "navigation", {"navigation": {"max_speed_m_s": 0.04, "position_gain": 0.4}}, "tab", 10.1
    )
    assert runtime.snapshot()["plan"]["body"][0] == pytest.approx(0.04)
    assert runtime.snapshot()["plan"]["body"][0] < before
    assert runtime.target.x == 0.5
    assert not runtime.active
    assert runtime.launch_limits.max_speed_m_s == 0.12


@pytest.mark.parametrize(
    "values",
    [
        {},
        [],
        {"max_speed_m_s": 0},
        {"max_speed_m_s": True},
        {"max_speed_m_s": "0.1"},
        {"position_gain": math.nan},
        {"yaw_gain": math.inf},
        {"max_speed_counts": 100},
        {"max_wheel_rad_s": 100},
        {"max_speed_m_s": 0.06, "yaw_gain": -1},
        {"max_yaw_rad_s": 100},
        {"wheel_accel_rad_s2": 100},
    ],
)
def test_invalid_navigation_is_atomic_and_does_not_refresh_lease(runtime, values):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    before = runtime.limits
    with pytest.raises(ValueError):
        runtime.command("navigation", {"navigation": values}, "tab", 10.1)
    assert runtime.limits == before
    assert runtime.active
    assert runtime.lease_until == pytest.approx(10.76)


def test_navigation_changes_stop_hardware_until_explicit_reapply(settings):
    board = MotorBoard()
    runtime = DashboardRuntime(*settings, driver=board, allow_motion=True)
    runtime.tick(10)
    runtime.command("mode", {"mode": "hardware"}, "tab", 10.001)
    set_target(runtime, now=10.002)
    runtime.command("apply", {}, "tab", 10.003)
    runtime.tick(10.1)
    assert any(board.commands[-1])
    runtime.command("navigation", {"navigation": {"max_speed_m_s": 0.06}}, "tab", 10.11)
    runtime.tick(10.15)
    assert not runtime.active
    assert board.commands[-1] == (0, 0, 0, 0)
    runtime.command("heartbeat", {}, "tab", 10.2)
    assert not runtime.active


def test_navigation_reset_restores_launch_values_and_disarms(runtime):
    original = runtime.limits
    runtime.command("navigation", {"navigation": {"max_speed_m_s": 0.05}}, "tab", 10.01)
    set_target(runtime, now=10.02)
    runtime.command("apply", {}, "tab", 10.03)
    runtime.command("navigation_reset", {}, "tab", 10.04)
    assert runtime.limits == original
    assert not runtime.active
    assert runtime.target.x == 0.5


def test_other_tab_cannot_retune_active_move(runtime):
    set_target(runtime)
    runtime.command("apply", {}, "tab", 10.01)
    with pytest.raises(ValueError, match="Another browser"):
        runtime.command("navigation", {"navigation": {"max_speed_m_s": 0.06}}, "other", 10.1)
    assert runtime.limits == runtime.launch_limits


def test_display_rounding_cannot_raise_exact_wheel_ceiling(runtime):
    ceiling = runtime.launch_limits.max_wheel_rad_s
    runtime.command(
        "navigation", {"navigation": {"max_wheel_rad_s": ceiling * (1 + 1e-12)}}, "tab", 10.01
    )
    assert runtime.limits.max_wheel_rad_s == ceiling
    assert runtime.snapshot()["navigation_bounds"]["max_wheel_rad_s"]["max"] == ceiling
