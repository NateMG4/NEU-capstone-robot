import http.client
import json
import threading
import time
from contextlib import ExitStack

import pytest

from capstone_base import control_dashboard as control
from capstone_base import dashboard as wheels_dashboard
from capstone_base import pan_tilt_dashboard as servo_dashboard
from capstone_base.servo import FakeServoController


@pytest.fixture
def hub_server():
    wheels = wheels_dashboard.DashboardRuntime(
        *wheels_dashboard.load_settings("config/odometry-demo.json")
    )
    servos = servo_dashboard.PanTiltRuntime(servo_dashboard.load_settings())
    with ExitStack() as stack:
        servos.controllers = {
            name: stack.enter_context(FakeServoController(config.servo))
            for name, config in servos.settings.items()
        }
        now = time.monotonic()
        wheels.tick(now)
        servos.tick(now)
        hub = control.ControlDashboard(wheels, servos)
        server = control.make_server(hub, "127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield hub, server.server_port
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


def request(port, path, *, payload=None, token=None, origin=None):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
    headers = {"Content-Type": "application/json"}
    if token is not None:
        headers["X-Control-Token"] = token
    if origin is not None:
        headers["Origin"] = origin
    try:
        conn.request(
            "POST" if payload is not None else "GET",
            path,
            body=json.dumps(payload) if payload is not None else None,
            headers=headers,
        )
        response = conn.getresponse()
        return response.status, dict(response.getheaders()), response.read()
    finally:
        conn.close()


def start_moves(hub):
    now = time.monotonic()
    hub.wheels.command("target", {"x": 0.5, "y": 0, "heading_deg": 0}, "wheel-tab", now)
    hub.wheels.command("apply", {}, "wheel-tab", now)
    hub.servos.command(
        {"action": "move", "client_id": "servo-tab", "yaw_deg": 15, "pitch_deg": 0}, now
    )
    hub.servos.tick(now)
    assert hub.wheels.active and hub.servos.active


def test_main_page_and_embedded_assets_are_routed_on_one_origin(hub_server):
    hub, port = hub_server
    for path, expected in (
        ("/", b"Control center"),
        ("/app.js", b"DashboardTabs"),
        ("/tabs.mjs", b"class DashboardTabs"),
        ("/style.css", b"@media"),
        ("/wheels/", b"Mecanum Lab"),
        ("/wheels/app.js", b'fetch("./api/state"'),
        ("/wheels/style.css", b"@media"),
        ("/pan-tilt/", b"Pan / Tilt"),
        ("/pan-tilt/app.js", b'from "./controls.mjs"'),
        ("/pan-tilt/controls.mjs", b"controlState"),
        ("/pan-tilt/style.css", b"@media"),
    ):
        status, headers, body = request(port, path)
        assert status == 200
        assert expected in body
        assert "frame-ancestors 'self'" in headers["Content-Security-Policy"]
    for path in ("/wheels", "/pan-tilt"):
        status, headers, _ = request(port, path)
        assert status == 308
        assert headers["Location"] == path + "/"
    for path in ("/../../pyproject.toml", "/wheels/../pan-tilt/", "/pan-tilt/unknown"):
        assert request(port, path)[0] == 404
    assert not hub.wheels.active
    assert not hub.servos.active


def test_each_subdashboard_retains_its_state_and_command_token(hub_server):
    hub, port = hub_server
    main = json.loads(request(port, "/api/state")[2])
    wheels = json.loads(request(port, "/wheels/api/state")[2])
    servos = json.loads(request(port, "/pan-tilt/api/state")[2])
    assert main["control_token"] == hub.token
    assert wheels["control_token"] == hub.wheels.token
    assert servos["control_token"] == hub.servos.token
    assert len({hub.token, hub.wheels.token, hub.servos.token}) == 3
    command = {"action": "stop", "client_id": "test"}
    assert request(port, "/wheels/api/command", payload=command, token=hub.token)[0] == 403
    assert request(port, "/pan-tilt/api/command", payload=command, token=hub.wheels.token)[0] == 403
    assert request(port, "/wheels/api/command", payload=command, token=hub.wheels.token)[0] == 200
    assert request(port, "/pan-tilt/api/command", payload=command, token=hub.servos.token)[0] == 200


def test_switch_stops_only_the_view_being_left_and_never_applies_a_target(hub_server):
    hub, port = hub_server
    start_moves(hub)
    payload = {"action": "switch", "client_id": "hub", "from": "wheels", "to": "pan-tilt"}
    assert request(port, "/api/command", payload=payload, token=hub.token)[0] == 200
    assert not hub.wheels.active
    assert hub.wheels.target.x == 0.5
    assert hub.servos.active
    payload.update({"from": "pan-tilt", "to": "wheels"})
    assert request(port, "/api/command", payload=payload, token=hub.token)[0] == 200
    assert hub.servos.pending == {"action": "stop"}
    hub.servos.tick(time.monotonic())
    assert not hub.servos.active
    assert not hub.wheels.active
    assert all(c.status.torque_enabled for c in hub.servos.controllers.values())


def test_stop_all_queues_both_and_origin_and_token_checks_remain_required(hub_server):
    hub, port = hub_server
    start_moves(hub)
    payload = {"action": "stop", "client_id": "hub"}
    assert request(port, "/api/command", payload=payload)[0] == 403
    assert (
        request(
            port,
            "/api/command",
            payload=payload,
            token=hub.token,
            origin="https://unrelated.example",
        )[0]
        == 403
    )
    assert hub.wheels.active and hub.servos.active
    assert request(port, "/api/command", payload=payload, token=hub.token)[0] == 200
    assert not hub.wheels.active
    assert hub.servos.pending == {"action": "stop"}
    hub.servos.tick(time.monotonic())
    assert not hub.servos.active


@pytest.mark.parametrize(
    "payload",
    [
        [],
        {},
        {"action": "apply", "client_id": "hub"},
        {"action": "switch", "client_id": "hub", "from": "wheels", "to": "unknown"},
        {"action": "switch", "client_id": "hub", "to": "pan-tilt"},
    ],
)
def test_invalid_hub_commands_are_rejected_without_changing_motion(hub_server, payload):
    hub, port = hub_server
    start_moves(hub)
    assert request(port, "/api/command", payload=payload, token=hub.token)[0] == 400
    assert hub.wheels.active and hub.servos.active


def test_standalone_dashboards_still_disallow_embedding(hub_server):
    hub, _ = hub_server
    for module, runtime in ((wheels_dashboard, hub.wheels), (servo_dashboard, hub.servos)):
        server = module.make_server(runtime, "127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            status, headers, _ = request(server.server_port, "/")
            assert status == 200
            assert "frame-ancestors 'none'" in headers["Content-Security-Policy"]
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)


@pytest.mark.parametrize(
    "arguments",
    [
        ["--wheel-allow-motion"],
        ["--servo-allow-motion"],
        ["--port", "0"],
        ["--servo-backend", "buslinker"],
    ],
)
def test_invalid_launch_never_opens_hardware_or_sockets(monkeypatch, arguments):
    def forbidden(*args, **kwargs):
        pytest.fail("Invalid configuration must fail before runtime or server creation")

    monkeypatch.setattr(control, "HiwonderController", forbidden)
    monkeypatch.setattr(control, "make_server", forbidden)
    with pytest.raises(SystemExit) as error:
        control.main(arguments)
    assert error.value.code == 2


@pytest.mark.parametrize("destination", ["wheels", "pan-tilt"])
def test_leaving_combined_stops_both_systems(hub_server, destination):
    hub, port = hub_server
    start_moves(hub)
    payload = {"action": "switch", "client_id": "hub", "from": "combined", "to": destination}
    assert request(port, "/api/command", payload=payload, token=hub.token)[0] == 200
    assert not hub.wheels.active
    hub.servos.tick(time.monotonic())
    assert not hub.servos.active
    assert all(c.status.torque_enabled for c in hub.servos.controllers.values())


def test_entering_combined_stops_previous_view_without_starting_motion(hub_server):
    hub, port = hub_server
    start_moves(hub)
    payload = {"action": "switch", "client_id": "hub", "from": "wheels", "to": "combined"}
    assert request(port, "/api/command", payload=payload, token=hub.token)[0] == 200
    assert not hub.wheels.active
    assert hub.servos.active
