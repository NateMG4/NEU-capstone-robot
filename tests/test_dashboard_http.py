import http.client
import json
import threading
import time

import pytest

from capstone_base.dashboard import DashboardRuntime, load_settings, make_server


@pytest.fixture
def dashboard_server():
    runtime = DashboardRuntime(*load_settings("config/odometry-demo.json"))
    runtime.tick(time.monotonic())
    server = make_server(runtime, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield runtime, server.server_port
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)


def request(port, method, path, data=None, headers=None):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
    try:
        conn.request(method, path, body=data, headers=headers or {})
        response = conn.getresponse()
        return response.status, response.getheaders(), response.read()
    finally:
        conn.close()


def test_page_assets_state_and_no_directory_serving(dashboard_server):
    runtime, port = dashboard_server
    for path, expected in (
        ("/", b"Mecanum Lab"),
        ("/app.js", b"/api/state"),
        ("/style.css", b"@media"),
    ):
        status, headers, data = request(port, "GET", path)
        assert status == 200
        assert expected in data
        assert ("Cache-Control", "no-store") in headers
    status, _, data = request(port, "GET", "/api/state")
    assert status == 200
    state = json.loads(data)
    assert state["control_token"] == runtime.token
    assert state["mode"] == "simulation"
    assert state["camera_connected"] is False
    assert request(port, "GET", "/../../pyproject.toml")[0] == 404


def test_commands_need_token_and_same_origin(dashboard_server):
    runtime, port = dashboard_server
    data = json.dumps({"action": "apply", "client_id": "browser"})
    assert request(port, "POST", "/api/command", data)[0] == 403
    headers = {
        "X-Control-Token": runtime.token,
        "Content-Type": "application/json",
        "Origin": "https://unrelated.example",
    }
    assert request(port, "POST", "/api/command", data, headers)[0] == 403
    assert not runtime.active


def test_target_preview_invalid_payload_and_explicit_apply(dashboard_server):
    runtime, port = dashboard_server
    headers = {"X-Control-Token": runtime.token, "Content-Type": "application/json"}
    target = json.dumps(
        {"action": "target", "client_id": "browser", "x": 0.3, "y": -0.2, "heading_deg": 30}
    )
    assert request(port, "POST", "/api/command", target, headers)[0] == 200
    assert runtime.target.x == 0.3
    assert not runtime.active
    for invalid in ("[]", '{"action":"target","client_id":"browser","x":NaN}', "{"):
        assert request(port, "POST", "/api/command", invalid, headers)[0] == 400
        assert not runtime.active
    apply = json.dumps({"action": "apply", "client_id": "browser"})
    assert request(port, "POST", "/api/command", apply, headers)[0] == 200
    assert runtime.active
    stop = json.dumps({"action": "stop", "client_id": "other-browser"})
    assert request(port, "POST", "/api/command", stop, headers)[0] == 200
    assert not runtime.active


def test_navigation_http_updates_and_validates_limits(dashboard_server):
    runtime, port = dashboard_server
    headers = {"X-Control-Token": runtime.token, "Content-Type": "application/json"}
    update = json.dumps(
        {
            "action": "navigation",
            "client_id": "browser",
            "navigation": {"max_speed_m_s": 0.06, "max_yaw_rad_s": 0.2},
        }
    )
    assert request(port, "POST", "/api/command", update, headers)[0] == 200
    status, _, body = request(port, "GET", "/api/state")
    assert status == 200
    state = json.loads(body)
    assert state["limits"]["max_speed_m_s"] == 0.06
    assert state["launch_limits"]["max_speed_m_s"] == 0.12
    assert (
        state["navigation_bounds"]["max_wheel_rad_s"]["max"]
        == runtime.launch_limits.max_wheel_rad_s
    )
    bad = json.dumps(
        {"action": "navigation", "client_id": "browser", "navigation": {"max_wheel_rad_s": 100}}
    )
    assert request(port, "POST", "/api/command", bad, headers)[0] == 400
    assert runtime.limits.max_speed_m_s == 0.06
    reset = json.dumps({"action": "navigation_reset", "client_id": "browser"})
    assert request(port, "POST", "/api/command", reset, headers)[0] == 200
    assert runtime.limits == runtime.launch_limits


def test_fault_report_and_recovery_queue_are_available_over_http(dashboard_server):
    runtime, port = dashboard_server
    runtime.trip(ValueError("Test encoder fault"))
    headers = {"X-Control-Token": runtime.token, "Content-Type": "application/json"}
    body = json.dumps({"action": "recover", "client_id": "browser"})
    assert request(port, "POST", "/api/command", body)[0] == 403
    assert not runtime.recovery_requested
    assert request(port, "POST", "/api/command", body, headers)[0] == 200
    state = json.loads(request(port, "GET", "/api/state")[2])
    assert state["recovery_pending"]
    assert state["fault_report"]["message"] == "Test encoder fault"
    assert state["encoder_configuration"]["counts_per_wheel_rev"] == [1000] * 4
    # The handler only queues intent; the control thread performs recovery.
    runtime.recover()
    state = json.loads(request(port, "GET", "/api/state")[2])
    assert state["fault"] is None
    assert state["pose_uncertain"]
    assert not state["active"]
