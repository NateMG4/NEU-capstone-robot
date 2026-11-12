import http.client
import json
import threading
import time
from contextlib import ExitStack
from dataclasses import replace

import pytest
from test_hiwonder_serial import Clock, RecordingChainSerial

from capstone_base import pan_tilt_dashboard as dashboard
from capstone_base.hiwonder_serial import LX224Bus
from capstone_base.servo import FakeServoController


@pytest.fixture
def bench():
    clock = Clock()
    runtime = dashboard.PanTiltRuntime(dashboard.load_settings(), clock=clock)
    with ExitStack() as stack:
        runtime.controllers = {
            name: stack.enter_context(FakeServoController(config.servo))
            for name, config in runtime.settings.items()
        }
        runtime.tick(clock())
        yield runtime, clock


def command(runtime, clock, action, client="browser", **kwargs):
    runtime.command({"action": action, "client_id": client, **kwargs}, clock())
    runtime.tick(clock())


def advance(runtime, clock, seconds, *, heartbeat=True):
    clock.now += seconds
    if heartbeat:
        runtime.command({"action": "heartbeat", "client_id": "browser"}, clock())
    runtime.tick(clock())


def test_startup_only_reads_and_default_simulation_never_opens_serial(bench):
    runtime, _ = bench
    assert runtime.snapshot()["backend"] == "simulation"
    assert runtime.snapshot()["connected"]
    assert all(c.history == [] for c in runtime.controllers.values())
    assert not runtime.active


def test_calibration_maps_direction_and_independent_limits(tmp_path):
    data = json.loads(open("config/pan-tilt.example.json").read())
    data["pan"].update(direction=-1, min_angle_deg=115, max_angle_deg=130)
    path = tmp_path / "axes.json"
    path.write_text(json.dumps(data))
    settings = dashboard.load_settings(path, hardware=True)
    assert settings["pan"].joint(125) == -5
    assert settings["pan"].raw(-5) == 125
    assert settings["pan"].report()["joint_limits_deg"] == [-10, 5]
    assert settings["tilt"].report()["joint_limits_deg"] is None
    data["tilt"]["servo_id"] = 1
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="distinct"):
        dashboard.load_settings(path)


@pytest.mark.parametrize(
    "field,value",
    [("direction", True), ("direction", 0), ("zero_deg", 240), ("zero_deg", float("nan"))],
)
def test_bad_axis_calibration_is_rejected(field, value):
    with pytest.raises(ValueError):
        replace(dashboard.load_settings()["pan"], **{field: value})


def test_large_target_is_split_into_bounded_verified_steps(bench):
    runtime, clock = bench
    command(runtime, clock, "move", yaw_deg=16, pitch_deg=-11)
    assert runtime.active
    for _ in range(10):
        if not runtime.active:
            break
        advance(runtime, clock, 1.2)
    assert runtime.fault is None
    assert not runtime.active
    assert "within tolerance" in runtime.message
    for name, goal in (("pan", 136), ("tilt", 109)):
        controller = runtime.controllers[name]
        assert controller.status.position_deg == pytest.approx(goal, abs=2)
        positions = [120] + [item[1] * 0.24 for item in controller.history if item[0] == "move"]
        assert len(positions) > 2
        assert all(abs(a - b) <= 5 for a, b in zip(positions, positions[1:], strict=False))
        assert controller.history[-1] == ("stop",)
        assert controller.status.torque_enabled


def test_jog_uses_fresh_axis_feedback_and_neutral_is_explicit(bench):
    runtime, clock = bench
    runtime.controllers["tilt"].status = replace(
        runtime.controllers["tilt"].status, position_deg=123.84
    )
    command(runtime, clock, "jog", axis="tilt", delta_deg=1, duration_s=1)
    assert runtime.controllers["pan"].history == []
    assert runtime.controllers["tilt"].status.position_deg == pytest.approx(124.8)
    advance(runtime, clock, 1.2)
    command(runtime, clock, "neutral")
    advance(runtime, clock, 1.2)
    assert runtime.controllers["tilt"].status.position_deg == 120


@pytest.mark.parametrize(
    "payload",
    [
        {"yaw_deg": 50, "pitch_deg": 0},
        {"yaw_deg": 0, "pitch_deg": -50},
        {"yaw_deg": float("nan"), "pitch_deg": 0},
        {"yaw_deg": 0, "pitch_deg": True},
        {"yaw_deg": 0, "pitch_deg": 0, "duration_s": 0.1},
    ],
)
def test_invalid_targets_do_not_write_or_queue(bench, payload):
    runtime, clock = bench
    with pytest.raises(ValueError):
        runtime.command({"action": "move", "client_id": "browser", **payload}, clock())
    assert runtime.pending is None
    assert all(c.history == [] for c in runtime.controllers.values())


@pytest.mark.parametrize(
    "change",
    [
        {"temperature_c": 61},
        {"voltage_v": 5.9},
        {"mode": 1},
        {"torque_enabled": False},
    ],
)
def test_entire_chain_preflight_before_first_move(bench, change):
    runtime, clock = bench
    controller = runtime.controllers["tilt"]
    controller.status = replace(controller.status, **change)
    command(runtime, clock, "move", yaw_deg=3, pitch_deg=2)
    assert runtime.fault
    assert not runtime.active
    assert all(c.history == [] for c in runtime.controllers.values())


def test_other_browser_cannot_take_over_but_can_stop(bench):
    runtime, clock = bench
    command(runtime, clock, "move", yaw_deg=20, pitch_deg=10)
    with pytest.raises(ValueError, match="Stop or wait"):
        runtime.command({"action": "neutral", "client_id": "other"}, clock())
    command(runtime, clock, "stop", client="other")
    assert not runtime.active
    assert runtime.owner is None
    assert all(c.history[-1] == ("stop",) for c in runtime.controllers.values())


def test_only_owner_heartbeat_keeps_motion_alive_and_expiry_stops(bench):
    runtime, clock = bench
    command(runtime, clock, "move", yaw_deg=20, pitch_deg=10)
    clock.now = 2.6
    runtime.command({"action": "heartbeat", "client_id": "spectator"}, clock())
    runtime.tick(clock())
    assert not runtime.active
    assert "heartbeat lost" in runtime.message
    assert all(c.history[-1] == ("stop",) for c in runtime.controllers.values())


def test_stale_queued_target_never_starts_and_stop_cancels_pending_target(bench):
    runtime, clock = bench
    payload = {"action": "move", "client_id": "browser", "yaw_deg": 5, "pitch_deg": 5}
    runtime.command(payload, clock())
    advance(runtime, clock, 2.6, heartbeat=False)
    assert not any(item[0] == "move" for c in runtime.controllers.values() for item in c.history)
    runtime.command(payload, clock())
    command(runtime, clock, "stop")
    assert not runtime.active
    assert not any(item[0] == "move" for c in runtime.controllers.values() for item in c.history)


def test_slow_preflight_cannot_start_move_after_heartbeat_expiry(bench, monkeypatch):
    runtime, clock = bench
    initialize = runtime.controllers["tilt"].initialize

    def slow_initialize():
        clock.now += 3
        return initialize()

    monkeypatch.setattr(runtime.controllers["tilt"], "initialize", slow_initialize)
    command(runtime, clock, "move", yaw_deg=5, pitch_deg=5)
    assert "expired during servo checks" in runtime.fault
    assert not runtime.active
    assert not any(item[0] == "move" for c in runtime.controllers.values() for item in c.history)


def test_step_plan_uses_position_returned_by_initialization(bench, monkeypatch):
    runtime, clock = bench
    controller = runtime.controllers["pan"]
    initialize = controller.initialize

    def settling_initialize():
        controller.status = replace(controller.status, position_deg=120.24)
        return initialize()

    monkeypatch.setattr(controller, "initialize", settling_initialize)
    command(runtime, clock, "move", yaw_deg=-15, pitch_deg=0)
    assert runtime.fault is None
    assert runtime.active
    assert runtime.step_targets["pan"] == pytest.approx(117.36)


def test_step_plan_leaves_room_for_drift_at_final_driver_read(bench, monkeypatch):
    runtime, clock = bench
    controller = runtime.controllers["pan"]
    move = controller.move_to

    def drifting_move(angle, duration):
        controller.status = replace(controller.status, position_deg=120.48)
        move(angle, duration)

    monkeypatch.setattr(controller, "move_to", drifting_move)
    command(runtime, clock, "move", yaw_deg=-15, pitch_deg=0)
    assert runtime.fault is None
    assert runtime.active
    assert abs(controller.status.position_deg - 120.48) <= controller.config.max_step_deg


def test_feedback_remains_fresh_while_motion_fault_is_latched(bench):
    runtime, clock = bench
    runtime._trip(ValueError("test movement fault"))
    controller = runtime.controllers["pan"]
    controller.status = replace(controller.status, position_deg=121.92, torque_enabled=False)
    advance(runtime, clock, 0.6)
    state = runtime.snapshot()
    assert state["fault"] == "test movement fault"
    assert state["axes"]["pan"]["status"]["position_deg"] == pytest.approx(121.92)
    assert state["axes"]["pan"]["status"]["torque_enabled"] is False
    assert state["axes"]["pan"]["age_s"] == 0
    assert all(c.history == [] for c in runtime.controllers.values())


def test_release_stops_both_axes_and_only_releases_selected_axis(bench):
    runtime, clock = bench
    command(runtime, clock, "move", yaw_deg=20, pitch_deg=10)
    command(runtime, clock, "torque", client="other", axis="tilt", enabled=False)
    assert not runtime.active
    assert not runtime.controllers["tilt"].status.torque_enabled
    assert runtime.controllers["pan"].status.torque_enabled
    assert runtime.controllers["pan"].history[-1] == ("stop",)
    command(runtime, clock, "torque", axis="tilt", enabled=True)
    assert runtime.controllers["tilt"].status.torque_enabled


def test_explicit_release_is_not_discarded_when_heartbeat_expires(bench):
    runtime, clock = bench
    command(runtime, clock, "move", yaw_deg=20, pitch_deg=10)
    clock.now = 3
    command(runtime, clock, "torque", client="other", axis="both", enabled=False)
    assert not runtime.active
    assert all(not c.status.torque_enabled for c in runtime.controllers.values())


def test_partial_chain_write_failure_stops_both_axes(bench, monkeypatch):
    runtime, clock = bench

    def fail(*args):
        raise OSError("tilt command disconnected")

    monkeypatch.setattr(runtime.controllers["tilt"], "move_to", fail)
    command(runtime, clock, "move", yaw_deg=3, pitch_deg=3)
    assert "tilt command disconnected" in runtime.fault
    assert not runtime.active
    assert any(item[0] == "move" for item in runtime.controllers["pan"].history)
    assert all(c.history[-1] == ("stop",) for c in runtime.controllers.values())


def test_target_readback_failure_latches_and_requires_read_only_recovery(bench):
    runtime, clock = bench
    command(runtime, clock, "move", yaw_deg=10, pitch_deg=10)
    runtime.controllers["pan"].status = replace(runtime.controllers["pan"].status, position_deg=120)
    advance(runtime, clock, 1.2)
    assert "target not reached" in runtime.fault
    report = runtime.snapshot()["fault_report"]["axes"]["pan"]
    assert report["goal_position_deg"] == pytest.approx(130.08)
    assert report["step_target_position_deg"] == pytest.approx(122.88)
    assert report["feedback_position_deg"] == 120
    assert "servo target 122.88°" in runtime.fault
    assert "feedback 120.00°" in runtime.fault
    assert not runtime.active
    assert all(c.history[-1] == ("stop",) for c in runtime.controllers.values())
    with pytest.raises(ValueError, match="clear fault"):
        command(runtime, clock, "neutral")
    before = [list(c.history) for c in runtime.controllers.values()]
    command(runtime, clock, "recover")
    assert not runtime.active
    assert runtime.fault is None
    assert before == [c.history for c in runtime.controllers.values()]


def test_health_failure_during_motion_stops_both(bench):
    runtime, clock = bench
    command(runtime, clock, "move", yaw_deg=20, pitch_deg=10)
    runtime.controllers["tilt"].status = replace(
        runtime.controllers["tilt"].status, temperature_c=70
    )
    advance(runtime, clock, 0.6)
    assert runtime.fault
    assert not runtime.active
    assert all(c.history[-1] == ("stop",) for c in runtime.controllers.values())


def test_one_stop_failure_does_not_skip_second_servo(bench, monkeypatch):
    runtime, clock = bench

    def fail():
        raise OSError("test disconnect")

    monkeypatch.setattr(runtime.controllers["pan"], "stop", fail)
    command(runtime, clock, "stop")
    assert "Stop unconfirmed" in runtime.fault
    assert runtime.controllers["tilt"].history[-1] == ("stop",)


@pytest.fixture
def hardware_bench():
    clock = Clock()
    port = RecordingChainSerial()
    settings = dashboard.load_settings()
    runtime = dashboard.PanTiltRuntime(settings, hardware=True, clock=clock)
    with LX224Bus(settings["pan"].servo, port=port, clock=port.clock) as bus:
        with ExitStack() as stack:
            runtime.controllers = {
                name: stack.enter_context(bus.controller(config.servo))
                for name, config in settings.items()
            }
            runtime.tick(clock())
            yield runtime, clock, port


def test_hardware_reads_chain_and_motion_permission_gate(hardware_bench):
    runtime, clock, port = hardware_bench
    assert {p[2] for p in port.packets} == {1, 2}
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets)
    with pytest.raises(ValueError, match="allow-motion"):
        command(runtime, clock, "move", yaw_deg=2, pitch_deg=2)
    with pytest.raises(ValueError, match="allow-motion"):
        command(runtime, clock, "torque", enabled=True)
    command(runtime, clock, "torque", axis="tilt", enabled=False)
    assert port.nodes[2]["torque"] == 0
    assert port.nodes[1]["torque"] == 1


def test_hardware_targets_route_to_distinct_ids_and_fault_stops_chain(hardware_bench):
    runtime, clock, port = hardware_bench
    runtime.allow_motion = True
    command(runtime, clock, "move", yaw_deg=2, pitch_deg=-3)
    assert [(p[2], int.from_bytes(p[5:7], "little")) for p in port.packets if p[4] == 1] == [
        (1, 508),
        (2, 488),
    ]
    port.read_error = True
    advance(runtime, clock, 0.6)
    assert runtime.fault
    assert not runtime.active
    assert {p[2] for p in port.packets if p[4] == 12} == {1, 2}
    assert not any(p[4] == 13 for p in port.packets)  # No persistent writes.


def test_uncalibrated_hardware_template_supports_read_only_refresh(tmp_path):
    settings = dashboard.load_settings("config/pan-tilt.example.json", hardware=True)
    clock = Clock()
    runtime = dashboard.PanTiltRuntime(settings, hardware=True, allow_motion=True, clock=clock)
    with ExitStack() as stack:
        runtime.controllers = {
            name: stack.enter_context(FakeServoController(config.servo))
            for name, config in settings.items()
        }
        runtime.tick(clock())
        command(runtime, clock, "recover")
        assert runtime.fault is None
        with pytest.raises(ValueError, match="measured"):
            command(runtime, clock, "neutral")
        assert all(c.history == [] for c in runtime.controllers.values())


def test_cli_rejects_implicit_hardware_config_and_invalid_port():
    with pytest.raises(ValueError, match="Hardware requires"):
        dashboard.load_settings(hardware=True)
    for args in (["--backend", "buslinker"], ["--allow-motion"], ["--port", "0"]):
        with pytest.raises(SystemExit) as error:
            dashboard.main(args)
        assert error.value.code == 2


@pytest.mark.parametrize("axes", ("both", "pan", "tilt"))
def test_worker_owns_one_bus_and_read_only_shutdown_never_writes_actuators(monkeypatch, axes):
    names = dashboard.selected_axes(axes)
    settings = dashboard.load_settings()
    ids = {settings[name].servo.servo_id for name in names}
    port = RecordingChainSerial(ids=ids)
    ready = threading.Event()
    bus_factory = dashboard.LX224Bus
    opened = []

    def make_bus(config):
        opened.append(config)
        return bus_factory(config, port=port, clock=port.clock)

    monkeypatch.setattr(dashboard, "LX224Bus", make_bus)
    runtime = dashboard.PanTiltRuntime(settings, hardware=True, axes=axes)
    original_tick = runtime.tick

    def tick(now):
        original_tick(now)
        ready.set()

    monkeypatch.setattr(runtime, "tick", tick)
    worker = threading.Thread(target=runtime.run, daemon=True)
    worker.start()
    try:
        assert ready.wait(2)
        assert runtime.snapshot()["connected"]
        assert set(runtime.controllers) == set(names)
        assert len(opened) == 1
    finally:
        runtime.quit.set()
        worker.join(2)
    assert not worker.is_alive()
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets)
    assert {p[2] for p in port.packets} == ids


def test_failed_port_open_is_reported_without_reconnection(monkeypatch):
    attempts = []

    def fail(config):
        attempts.append(config)
        raise OSError("Serial port missing")

    monkeypatch.setattr(dashboard, "LX224Bus", fail)
    runtime = dashboard.PanTiltRuntime(dashboard.load_settings(), hardware=True)
    runtime.run()
    assert runtime.snapshot()["fault"] == "Serial port missing"
    assert not runtime.snapshot()["connected"]
    assert len(attempts) == 1


@pytest.fixture
def http_bench(bench):
    runtime, clock = bench
    server = dashboard.make_server(runtime, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def request(method, path, payload=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        try:
            conn.request(method, path, body=payload, headers=headers or {})
            response = conn.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            conn.close()

    try:
        yield runtime, clock, request
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)


def test_http_assets_state_and_no_directory_serving(http_bench):
    runtime, _, request = http_bench
    for path, expected in (
        ("/", b"Pan. Tilt. Test."),
        ("/app.js", b"/api/state"),
        ("/controls.mjs", b"controlState"),
        ("/style.css", b"@media"),
    ):
        status, headers, data = request("GET", path)
        assert status == 200
        assert expected in data
        assert headers["Cache-Control"] == "no-store"
    status, _, data = request("GET", "/api/state")
    assert status == 200
    assert json.loads(data)["control_token"] == runtime.token
    assert request("GET", "/../../pyproject.toml")[0] == 404


def test_http_requires_token_same_origin_json_and_queues_without_io(http_bench):
    runtime, _, request = http_bench
    payload = json.dumps({"action": "move", "client_id": "browser", "yaw_deg": 4, "pitch_deg": 2})
    headers = {"X-Control-Token": runtime.token, "Content-Type": "application/json"}
    assert request("POST", "/api/command", payload)[0] == 403
    assert (
        request(
            "POST", "/api/command", payload, {**headers, "Origin": "https://unrelated.example"}
        )[0]
        == 403
    )
    for invalid in (
        "[]",
        "null",
        "{",
        '{"action":"move","client_id":"browser","yaw_deg":NaN,"pitch_deg":0}',
    ):
        assert request("POST", "/api/command", invalid, headers)[0] == 400
    assert request("POST", "/api/command", payload, headers)[0] == 200
    assert runtime.pending
    assert all(c.history == [] for c in runtime.controllers.values())


@pytest.mark.parametrize("axes", ("both", "pan", "tilt"))
@pytest.mark.parametrize("test_mode", (False, True))
def test_http_live_worker_completes_explicit_move(axes, test_mode):
    runtime = dashboard.PanTiltRuntime(dashboard.load_settings(), axes=axes, test_mode=test_mode)
    server = dashboard.make_server(runtime, "127.0.0.1", 0)
    worker = threading.Thread(target=runtime.run, daemon=True)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    http_thread.start()
    conn = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)

    def state():
        conn.request("GET", "/api/state")
        response = conn.getresponse()
        assert response.status == 200
        return json.loads(response.read())

    try:
        deadline = time.monotonic() + 2
        current = state()
        while not current["connected"] and time.monotonic() < deadline:
            time.sleep(0.01)
            current = state()
        assert current["connected"]
        assert not current["active"]
        assert all(c.history == [] for c in runtime.controllers.values())
        headers = {"X-Control-Token": current["control_token"], "Content-Type": "application/json"}
        payload = {
            "action": "jog" if test_mode else "move",
            "client_id": "browser",
            "axis": runtime.axes[0] if test_mode else "both",
            "delta_deg": 1,
            "yaw_deg": 3,
            "pitch_deg": -2,
            "duration_s": 0.5,
        }
        conn.request(
            "POST",
            "/api/command",
            json.dumps(payload),
            headers,
        )
        response = conn.getresponse()
        assert response.status == 200
        response.read()
        deadline = time.monotonic() + 2
        current = state()
        while (current["active"] or current["pending"]) and time.monotonic() < deadline:
            time.sleep(0.05)
            current = state()
        assert not current["active"]
        assert current["fault"] is None
        for name, target in (("pan", 3), ("tilt", -2)):
            data = current["axes"][name]
            assert data["enabled"] == (name in runtime.axes)
            if data["enabled"]:
                if test_mode:
                    assert data["joint_deg"] is None
                    assert data["config"]["zero_deg"] is None
                    expected = 120.96 if name == runtime.axes[0] else 120
                    assert data["status"]["position_deg"] == pytest.approx(expected)
                else:
                    assert data["joint_deg"] == pytest.approx(target, abs=0.24)
            else:
                assert data["status"] is None
                assert data["joint_deg"] is None
        assert "Target reached" in current["message"]
    finally:
        conn.close()
        runtime.quit.set()
        worker.join(2)
        server.shutdown()
        server.server_close()
        http_thread.join(2)


@pytest.fixture(params=dashboard.AXES)
def single_hardware_bench(request):
    name = request.param
    other = "tilt" if name == "pan" else "pan"
    settings = dashboard.load_settings()
    # The unused axis has no limits, an incompatible duration, and the same ID.
    # None of these should prevent testing the single connected servo.
    settings[other] = replace(
        settings[other],
        servo=replace(
            settings[other].servo,
            servo_id=settings[name].servo.servo_id,
            min_angle_deg=None,
            max_angle_deg=None,
            min_duration_s=3,
            max_duration_s=4,
        ),
    )
    servo_id = settings[name].servo.servo_id
    port = RecordingChainSerial(ids=(servo_id,))
    clock = Clock()
    runtime = dashboard.PanTiltRuntime(
        settings, hardware=True, allow_motion=True, axes=name, clock=clock
    )
    with LX224Bus(settings[name].servo, port=port, clock=port.clock) as bus:
        with bus.controller(settings[name].servo) as controller:
            runtime.controllers = {name: controller}
            runtime.tick(clock())
            yield runtime, clock, port, name, other
    assert port.packets
    assert all(packet[2] == servo_id for packet in port.packets)
    assert not any(packet[4] == 13 for packet in port.packets)


def test_single_servo_status_does_not_require_other_axis(single_hardware_bench):
    runtime, _, port, name, other = single_hardware_bench
    state = runtime.snapshot()
    assert state["enabled_axes"] == [name]
    assert state["connected"] and state["available"]
    assert state["axes"][name]["enabled"]
    assert not state["axes"][other]["enabled"]
    assert state["axes"][other]["status"] is None
    assert state["fault"] is None
    assert all(packet[4] in (26, 27, 28, 30, 32) for packet in port.packets)


def test_single_servo_move_jog_neutral_and_torque(single_hardware_bench):
    runtime, clock, port, name, _ = single_hardware_bench
    target_field = "yaw_deg" if name == "pan" else "pitch_deg"
    servo_id = runtime.settings[name].servo.servo_id
    command(runtime, clock, "move", **{target_field: 12}, duration_s=0.5)
    for _ in range(5):
        if not runtime.active:
            break
        advance(runtime, clock, 0.7)
    assert not runtime.active
    assert runtime.fault is None
    assert port.nodes[servo_id]["position"] * 0.24 == pytest.approx(132, abs=2)
    command(runtime, clock, "jog", axis=name, delta_deg=-1, duration_s=0.5)
    advance(runtime, clock, 0.7)
    assert runtime.fault is None
    command(runtime, clock, "neutral", duration_s=0.5)
    for _ in range(5):
        if not runtime.active:
            break
        advance(runtime, clock, 0.7)
    assert not runtime.active
    assert port.nodes[servo_id]["position"] * 0.24 == pytest.approx(120, abs=2)
    command(runtime, clock, "torque", enabled=False)
    assert port.nodes[servo_id]["torque"] == 0
    command(runtime, clock, "torque", enabled=True)
    assert port.nodes[servo_id]["torque"] == 1
    command(runtime, clock, "stop")
    assert runtime.fault is None


@pytest.mark.parametrize(
    "action,values",
    [
        ("move", {"yaw_deg": 1, "pitch_deg": 1}),
        ("jog", {"delta_deg": 1}),
        ("neutral", {}),
        ("torque", {"enabled": False}),
        ("torque", {"enabled": True}),
    ],
)
def test_single_mode_rejects_commands_to_disabled_axis(single_hardware_bench, action, values):
    runtime, clock, port, _, other = single_hardware_bench
    before = len(port.packets)
    with pytest.raises(ValueError, match="disabled"):
        command(runtime, clock, action, axis=other, **values)
    assert runtime.pending is None
    assert len(port.packets) == before


def test_single_mode_fault_recovery_and_heartbeat_stop(single_hardware_bench):
    runtime, clock, port, name, _ = single_hardware_bench
    target_field = "yaw_deg" if name == "pan" else "pitch_deg"
    command(runtime, clock, "move", **{target_field: 10}, duration_s=0.5)
    port.read_error = True
    advance(runtime, clock, 0.6)
    assert runtime.fault
    assert not runtime.active
    assert not runtime.snapshot()["connected"]
    assert runtime.snapshot()["available"]  # The browser can still request recovery.
    assert any(packet[4] == 12 for packet in port.packets)
    port.read_error = False
    command(runtime, clock, "recover")
    assert runtime.fault is None
    assert not runtime.active
    command(runtime, clock, "neutral", duration_s=0.5)
    advance(runtime, clock, 2.6, heartbeat=False)
    assert not runtime.active
    assert "heartbeat lost" in runtime.message
    assert port.packets[-1][4] == 12


@pytest.mark.parametrize("axes", dashboard.AXES)
def test_single_mode_allows_same_configured_id_for_separate_sessions(tmp_path, axes):
    data = json.loads(open("config/pan-tilt.example.json").read())
    data["tilt"]["servo_id"] = data["pan"]["servo_id"]
    path = tmp_path / "single.json"
    path.write_text(json.dumps(data))
    settings = dashboard.load_settings(path, hardware=True, axes=axes)
    dashboard.PanTiltRuntime(settings, axes=axes)
    with pytest.raises(ValueError, match="distinct"):
        dashboard.PanTiltRuntime(settings, axes="both")
    with pytest.raises(ValueError, match="distinct"):
        dashboard.load_settings(path, axes="both")


@pytest.mark.parametrize("missing_id", (1, 2))
def test_both_mode_still_faults_if_one_selected_servo_is_absent(missing_id):
    settings = dashboard.load_settings()
    port = RecordingChainSerial(ids=({1, 2} - {missing_id}))
    runtime = dashboard.PanTiltRuntime(settings, hardware=True)
    with LX224Bus(settings["pan"].servo, port=port, clock=port.clock) as bus:
        with ExitStack() as stack:
            runtime.controllers = {
                name: stack.enter_context(bus.controller(config.servo))
                for name, config in settings.items()
            }
            runtime.tick(time.monotonic())
            assert runtime.fault
            assert not runtime.snapshot()["connected"]
            assert runtime.snapshot()["enabled_axes"] == ["pan", "tilt"]
    assert all(packet[4] in (26, 27, 28, 30, 32) for packet in port.packets)


@pytest.fixture(params=dashboard.AXES)
def test_mode_bench(request):
    name = request.param
    settings = dashboard.load_settings("config/pan-tilt.example.json", axes=name, test_mode=True)
    clock = Clock()
    runtime = dashboard.PanTiltRuntime(
        settings, hardware=True, allow_motion=True, axes=name, test_mode=True, clock=clock
    )
    config = runtime.settings[name].servo
    port = RecordingChainSerial(ids=(config.servo_id,))
    with LX224Bus(config, port=port, clock=port.clock) as bus:
        with bus.controller(config) as controller:
            runtime.controllers = {name: controller}
            runtime.tick(clock())
            yield runtime, clock, port, name
    assert all(packet[2] == config.servo_id for packet in port.packets)


def test_bring_up_jogs_without_neutral_or_travel_calibration(test_mode_bench):
    runtime, clock, port, name = test_mode_bench
    state = runtime.snapshot()
    assert state["test_mode"]
    assert state["jog_max_deg"] == 2
    assert state["axes"][name]["joint_deg"] is None
    assert state["axes"][name]["config"]["zero_deg"] is None
    assert state["axes"][name]["config"]["joint_limits_deg"] is None
    assert runtime.fault is None
    assert all(packet[4] in (26, 27, 28, 30, 32) for packet in port.packets)
    command(runtime, clock, "jog", axis=name, delta_deg=2, duration_s=0.5)
    for _ in range(4):
        if not runtime.active:
            break
        advance(runtime, clock, 0.7)
    assert runtime.fault is None
    assert not runtime.active
    assert runtime.feedback[name].position_deg == pytest.approx(121.92)
    assert all(abs(packet[5] | packet[6] << 8) <= 1000 for packet in port.packets if packet[4] == 1)


def test_test_mode_recovers_after_torque_release_and_manual_reposition(test_mode_bench):
    runtime, clock, port, name = test_mode_bench
    servo_id = runtime.settings[name].servo.servo_id
    command(runtime, clock, "torque", axis=name, enabled=False)
    assert port.nodes[servo_id]["torque"] == 0
    port.nodes[servo_id]["position"] = 167  # 40.08°, beyond the old example calibration.
    runtime._trip(ValueError("earlier movement failed after torque event"))
    before = len(port.packets)
    command(runtime, clock, "recover")
    assert runtime.fault is None
    assert not runtime.active
    assert port.nodes[servo_id]["torque"] == 0
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets[before:])
    command(runtime, clock, "torque", axis=name, enabled=True)
    assert runtime.fault is None
    assert port.nodes[servo_id]["torque"] == 1
    command(runtime, clock, "jog", axis=name, delta_deg=1, duration_s=0.5)
    advance(runtime, clock, 0.7)
    advance(runtime, clock, 0.7)
    assert runtime.fault is None
    assert runtime.feedback[name].position_deg == pytest.approx(41.04)


@pytest.mark.parametrize(
    "action,values",
    [
        ("neutral", {}),
        ("move", {"yaw_deg": 0, "pitch_deg": 0}),
        ("jog", {"delta_deg": 2.01}),
        ("jog", {"delta_deg": -2.01}),
    ],
)
def test_test_mode_rejects_absolute_targets_and_large_jogs(test_mode_bench, action, values):
    runtime, clock, port, name = test_mode_bench
    before = len(port.packets)
    with pytest.raises(ValueError):
        command(runtime, clock, action, axis=name, **values)
    assert runtime.pending is None
    assert len(port.packets) == before


def test_test_mode_keeps_protocol_bounds_and_health_checks(test_mode_bench):
    runtime, clock, port, name = test_mode_bench
    servo_id = runtime.settings[name].servo.servo_id
    port.nodes[servo_id]["position"] = 999
    command(runtime, clock, "jog", axis=name, delta_deg=1)
    assert runtime.fault
    assert not any(packet[4] == 1 for packet in port.packets)
    command(runtime, clock, "recover")
    assert runtime.fault is None
    port.nodes[servo_id]["temperature"] = 70
    command(runtime, clock, "jog", axis=name, delta_deg=-1)
    assert runtime.fault
    before = len(port.packets)
    command(runtime, clock, "recover")
    assert runtime.fault  # Health remains bad, but recovery does not send actuator writes.
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets[before:])


def test_calibrated_fault_recovery_does_not_require_current_position_inside_limits(bench):
    runtime, clock = bench
    runtime.controllers["pan"].status = replace(
        runtime.controllers["pan"].status, position_deg=40.08, torque_enabled=False
    )
    runtime._trip(ValueError("previous movement fault"))
    before = [list(c.history) for c in runtime.controllers.values()]
    command(runtime, clock, "recover")
    assert runtime.fault is None
    assert not runtime.active
    assert before == [c.history for c in runtime.controllers.values()]
    assert not runtime.feedback["pan"].torque_enabled


def test_test_mode_ignores_placeholder_calibration_without_mutating_config(tmp_path):
    data = json.loads(open("config/pan-tilt.example.json").read())
    data["pan"].update(zero_deg=0, min_angle_deg=-120, max_angle_deg=120)
    path = tmp_path / "uncalibrated.json"
    original = json.dumps(data)
    path.write_text(original)
    settings = dashboard.load_settings(path, axes="pan", test_mode=True)
    runtime = dashboard.PanTiltRuntime(settings, axes="pan", test_mode=True)
    assert runtime.settings["pan"].zero_deg is None
    assert runtime.settings["pan"].servo.min_angle_deg == 0
    assert runtime.settings["pan"].servo.max_angle_deg == 240
    assert runtime.settings["pan"].servo.max_step_deg == 2
    assert path.read_text() == original
    with pytest.raises(ValueError):
        dashboard.load_settings(path, axes="pan")


@pytest.fixture
def calibration_bench(tmp_path):
    data = json.loads(open("config/pan-tilt.example.json").read())
    data["tilt"].update(servo_id=1, zero_deg=None, min_duration_s=0.1, max_step_deg=10)
    path = tmp_path / "pan-tilt.local.json"
    path.write_text(json.dumps(data))
    settings = dashboard.load_settings(path, axes="tilt", test_mode=True)
    clock, port = Clock(), RecordingChainSerial(ids=(1,))
    runtime = dashboard.PanTiltRuntime(
        settings,
        hardware=True,
        allow_motion=True,
        axes="tilt",
        test_mode=True,
        config_path=path,
        clock=clock,
    )
    config = runtime.settings["tilt"].servo
    with LX224Bus(config, port=port, clock=port.clock) as bus:
        with bus.controller(config) as controller:
            runtime.controllers = {"tilt": controller}
            runtime.tick(clock())
            yield runtime, clock, port, path
    assert all(packet[2] == 1 for packet in port.packets)


def record_calibration(runtime, clock, port, neutral=167, end_a=125, end_b=230):
    for point, position in (("zero_deg", neutral), ("end_a", end_a), ("end_b", end_b)):
        port.nodes[1]["position"] = position
        command(runtime, clock, "capture_calibration", axis="tilt", point=point)


@pytest.mark.parametrize("direction", (1, -1))
def test_record_save_apply_and_move_pitch_id_one(calibration_bench, direction):
    runtime, clock, port, path = calibration_bench
    original = json.loads(path.read_text())
    record_calibration(runtime, clock, port)
    report = runtime.snapshot()["calibration"]["tilt"]
    assert report["points"]["zero_deg"] == pytest.approx(40.08)
    assert report["candidate"] is not None
    command(runtime, clock, "save_calibration", axis="tilt", direction=direction)
    assert runtime.fault is None
    saved = json.loads(path.read_text())
    assert saved["pan"] == original["pan"]
    assert saved["tilt"]["servo_id"] == 1
    assert saved["tilt"]["zero_deg"] == pytest.approx(40.08)
    assert saved["tilt"]["min_angle_deg"] == 30
    assert saved["tilt"]["max_angle_deg"] == pytest.approx(55.2)
    assert saved["tilt"]["direction"] == direction
    assert saved["tilt"]["min_duration_s"] == 0.1
    assert runtime.test_mode
    command(runtime, clock, "use_calibration")
    assert runtime.fault is None
    assert not runtime.test_mode
    assert runtime.settings["tilt"].servo.max_step_deg == 10
    assert runtime.controllers["tilt"].config == runtime.settings["tilt"].servo
    assert runtime.snapshot()["axes"]["tilt"]["joint_deg"] == pytest.approx(15.12 * direction)
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets)
    command(runtime, clock, "neutral", duration_s=0.2)
    for _ in range(8):
        if not runtime.active:
            break
        advance(runtime, clock, 0.4)
    assert runtime.fault is None
    assert not runtime.active
    assert runtime.feedback["tilt"].position_deg == pytest.approx(40.08)
    assert all(int.from_bytes(p[7:9], "little") == 200 for p in port.packets if p[4] == 1)
    reloaded = dashboard.load_settings(path, axes="tilt")
    assert reloaded["tilt"].zero_deg == pytest.approx(40.08)


def test_calibration_sorts_recorded_ends_and_recapture_requires_save(calibration_bench):
    runtime, clock, port, _ = calibration_bench
    record_calibration(runtime, clock, port, end_a=230, end_b=125)
    command(runtime, clock, "save_calibration", axis="tilt", direction=1)
    assert runtime.snapshot()["calibration"]["tilt"]["saved"]
    command(runtime, clock, "capture_calibration", axis="tilt", point="end_a")
    assert not runtime.snapshot()["calibration"]["tilt"]["saved"]
    with pytest.raises(ValueError, match="Save calibration"):
        command(runtime, clock, "use_calibration")


@pytest.mark.parametrize("neutral,end_a,end_b", [(167, 125, 125), (240, 125, 230)])
def test_invalid_recorded_calibration_cannot_overwrite_file(
    calibration_bench, neutral, end_a, end_b
):
    runtime, clock, port, path = calibration_bench
    original = path.read_text()
    record_calibration(runtime, clock, port, neutral, end_a, end_b)
    assert runtime.snapshot()["calibration"]["tilt"]["candidate"] is None
    with pytest.raises(ValueError):
        command(runtime, clock, "save_calibration", axis="tilt", direction=1)
    assert path.read_text() == original
    assert runtime.fault is None


def test_apply_calibration_requires_current_position_inside_new_limits(calibration_bench):
    runtime, clock, port, _ = calibration_bench
    record_calibration(runtime, clock, port)
    command(runtime, clock, "save_calibration", axis="tilt", direction=1)
    port.nodes[1]["position"] = 500
    command(runtime, clock, "use_calibration")
    assert runtime.fault
    assert runtime.test_mode
    assert runtime.settings["tilt"].zero_deg is None
    assert all(p[4] in (26, 27, 28, 30, 32) for p in port.packets)


def test_calibration_does_not_overwrite_changed_servo_id(calibration_bench):
    runtime, clock, port, path = calibration_bench
    record_calibration(runtime, clock, port)
    data = json.loads(path.read_text())
    data["tilt"]["servo_id"] = 2
    original = json.dumps(data)
    path.write_text(original)
    command(runtime, clock, "save_calibration", axis="tilt", direction=1)
    assert "Servo ID changed" in runtime.fault
    assert path.read_text() == original


def test_capture_calibration_is_protected_and_read_only_over_http(http_bench):
    runtime, clock, request = http_bench
    payload = json.dumps(
        {
            "action": "capture_calibration",
            "axis": "pan",
            "point": "zero_deg",
            "client_id": "browser",
        }
    )
    assert request("POST", "/api/command", payload)[0] == 403
    headers = {"X-Control-Token": runtime.token, "Content-Type": "application/json"}
    assert request("POST", "/api/command", payload, headers)[0] == 200
    runtime.tick(clock())
    assert runtime.snapshot()["calibration"]["pan"]["points"]["zero_deg"] == 120
    assert all(c.history == [] for c in runtime.controllers.values())
