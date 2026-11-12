"""Exercise real UDP input through the existing authenticated dashboard HTTP API."""

import http.client
import json
import socket
import threading
import time

from capstone_base.dashboard import DashboardRuntime, load_settings, make_server
from capstone_base.vision import VisionConfig, VisionReceiver


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for camera/control worker")


def test_udp_to_dashboard_alignment_freshness_and_no_automatic_motion():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(("127.0.0.1", 0))
        udp_port = reserve.getsockname()[1]
    receiver = VisionReceiver(VisionConfig(port=udp_port))
    runtime = DashboardRuntime(*load_settings("config/odometry-demo.json"), vision=receiver)
    server = make_server(runtime, "127.0.0.1", 0)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    control = threading.Thread(target=runtime.run, daemon=True)
    http_thread.start()
    control.start()
    receiver.start()
    try:
        wait_for(lambda: runtime.odometry["simulation"].time is not None)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(
                json.dumps(
                    {
                        "t": time.time(),
                        "frame": 1,
                        "robots": [{"id": 1, "x": 4, "y": 5, "heading": 90, "age": 0}],
                        "user": None,
                    }
                ).encode(),
                ("127.0.0.1", udp_port),
            )
            wait_for(lambda: receiver.accepted == 1)
            assert runtime.pose().x == 0
            assert not runtime.active
            connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
            try:
                body = json.dumps({"action": "camera_align", "client_id": "test"})
                connection.request(
                    "POST", "/api/command", body, {"Content-Type": "application/json"}
                )
                response = connection.getresponse()
                assert response.status == 403
                response.read()
                connection.request(
                    "POST",
                    "/api/command",
                    body,
                    {
                        "Content-Type": "application/json",
                        "X-Control-Token": runtime.token,
                    },
                )
                response = connection.getresponse()
                assert response.status == 200
                response.read()
                wait_for(lambda: runtime.camera_enabled["simulation"])
                connection.request("GET", "/api/state")
                response = connection.getresponse()
                state = json.loads(response.read())
                assert state["camera_connected"]
                assert state["frame_id"] == "floor"
                assert state["pose"]["x"] == 4
                assert state["encoder_pose"]["x"] == 0
                assert state["camera"]["configuration"]["robot_id"] == 1
                assert state["camera"]["enabled"]
                assert not state["active"]
                assert not any(state["applied_wheels"])
            finally:
                connection.close()
            sender.sendto(b"invalid json", ("127.0.0.1", udp_port))
            wait_for(lambda: receiver.rejected == 1)
        wait_for(lambda: not runtime.snapshot()["camera_connected"])
        assert runtime.camera_enabled["simulation"]
        assert runtime.pose().x == 4
        assert runtime.fault is None
    finally:
        runtime.quit.set()
        control.join(2)
        receiver.close()
        server.shutdown()
        server.server_close()
        http_thread.join(2)
    assert not receiver.thread.is_alive()
    assert not control.is_alive()
