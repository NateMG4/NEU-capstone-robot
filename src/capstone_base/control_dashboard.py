"""One local address for wheel/navigation and pan/tilt dashboards."""

import argparse
import json
import secrets
import signal
import threading
import time
from http.server import ThreadingHTTPServer
from importlib.resources import files
from pathlib import Path
from urllib.parse import urlsplit

from . import dashboard as wheels_dashboard
from . import pan_tilt_dashboard as servo_dashboard
from .dashboard_http import DashboardHandler
from .hiwonder_i2c import HiwonderController
from .vision import VisionConfig, VisionReceiver


class ControlDashboard:
    def __init__(self, wheels, servos):
        self.wheels = wheels
        self.servos = servos
        self.token = secrets.token_urlsafe(24)

    def snapshot(self):
        wheels, servos = self.wheels.snapshot(), self.servos.snapshot()
        return {
            "control_token": self.token,
            "dashboards": {
                "wheels": {
                    "label": "Wheels & navigation",
                    "path": "/wheels/",
                    "mode": wheels["mode"],
                    "active": wheels["active"],
                    "fault": wheels["fault"],
                    "message": wheels["status"],
                    "motion_allowed": wheels["hardware_motion_allowed"],
                    "hardware_available": wheels["hardware_available"],
                    "camera_connected": wheels["camera_connected"],
                },
                "pan-tilt": {
                    "label": "Pan & tilt",
                    "path": "/pan-tilt/",
                    "mode": servos["backend"],
                    "active": servos["active"],
                    "fault": servos["fault"],
                    "message": servos["message"],
                    "motion_allowed": self.servos.allow_motion,
                    "axes": list(self.servos.axes),
                    "test_mode": self.servos.test_mode,
                },
            },
        }

    def stop(self, name, client, now):
        if name == "wheels":
            self.wheels.command("stop", {}, client, now)
        else:
            self.servos.command({"action": "stop", "client_id": client}, now)

    def command(self, payload, now):
        if not isinstance(payload, dict):
            raise ValueError("Expected a JSON object")
        client = payload.get("client_id")
        if not isinstance(client, str) or not 1 <= len(client) <= 100:
            raise ValueError("A client_id is required")
        action = payload.get("action")
        if action == "switch":
            previous, selected = payload.get("from"), payload.get("to")
            if previous not in ("wheels", "pan-tilt", "combined") or selected not in (
                "wheels",
                "pan-tilt",
                "combined",
            ):
                raise ValueError("Choose wheels, pan-tilt, or combined")
            if previous != selected:
                for name in ("wheels", "pan-tilt") if previous == "combined" else (previous,):
                    self.stop(name, client, now)
        elif action == "stop":
            for name in ("wheels", "pan-tilt"):
                self.stop(name, client, now)
        else:
            raise ValueError("Unknown action")


def make_server(hub, host, port):
    routes = {
        "/wheels/": wheels_dashboard.make_handler(hub.wheels),
        "/pan-tilt/": servo_dashboard.make_handler(hub.servos),
    }

    class Handler(DashboardHandler):
        frame_ancestors = "'self'"

        def dispatch(self, method):
            # The subdashboard handlers share DashboardHandler's request/reply
            # interface. Reuse their endpoint logic without proxy sockets or a
            # second HTTP server, retaining each runtime's own command token.
            for prefix, handler in routes.items():
                if self.path.startswith(prefix):
                    original = self.path
                    self.path = "/" + original[len(prefix) :]
                    try:
                        getattr(handler, f"do_{method}")(self)
                    finally:
                        self.path = original
                    return True
            return False

        def do_GET(self):
            if self.dispatch("GET"):
                return
            path = urlsplit(self.path).path
            if path in ("/wheels", "/pan-tilt"):
                self.send_response(308)
                self.send_header("Location", path + "/")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            if path == "/api/state":
                self.reply(200, hub.snapshot())
                return
            assets = {
                "/": ("index.html", "text/html; charset=utf-8"),
                "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                "/style.css": ("style.css", "text/css; charset=utf-8"),
                "/tabs.mjs": ("tabs.mjs", "text/javascript; charset=utf-8"),
            }
            if path not in assets:
                self.reply(404, {"error": "Not found"})
                return
            name, mime = assets[path]
            self.reply(200, files("capstone_base").joinpath("control_web", name).read_bytes(), mime)

        def do_POST(self):
            if self.dispatch("POST"):
                return
            if self.path != "/api/command":
                self.reply(404, {"error": "Not found"})
                return
            origin = self.headers.get("Origin")
            if (
                origin and urlsplit(origin).netloc != self.headers.get("Host")
            ) or not secrets.compare_digest(self.headers.get("X-Control-Token", ""), hub.token):
                self.reply(403, {"error": "Send commands from the main dashboard"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("Invalid request size")
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected application/json")
                hub.command(json.loads(self.rfile.read(length)), time.monotonic())
            except (ValueError, TypeError, UnicodeError) as error:
                self.reply(400, {"error": str(error)})
                return
            self.reply(200, {"ok": True})

    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Combined wheel and pan/tilt control dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--wheel-backend", choices=("simulation", "hiwonder"), default="simulation")
    parser.add_argument("--wheel-config", type=Path, default=Path("config/odometry-demo.json"))
    parser.add_argument("--wheel-allow-motion", action="store_true")
    parser.add_argument("--vision-config", type=Path)
    parser.add_argument(
        "--servo-backend", choices=("simulation", "buslinker"), default="simulation"
    )
    parser.add_argument("--servo-config", type=Path)
    parser.add_argument("--servo-axes", choices=("both", "pan", "tilt"), default="both")
    parser.add_argument("--servo-test-mode", action="store_true")
    parser.add_argument("--servo-allow-motion", action="store_true")
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.port <= 65535:
            raise ValueError("Port must be between 1 and 65535")
        wheel_hardware = args.wheel_backend == "hiwonder"
        servo_hardware = args.servo_backend == "buslinker"
        if args.wheel_allow_motion and not wheel_hardware:
            raise ValueError("--wheel-allow-motion requires --wheel-backend hiwonder")
        if args.servo_allow_motion and not servo_hardware:
            raise ValueError("--servo-allow-motion requires --servo-backend buslinker")
        wheel_settings = wheels_dashboard.load_settings(args.wheel_config, wheel_hardware)
        servo_settings = servo_dashboard.load_settings(
            args.servo_config,
            hardware=servo_hardware,
            axes=args.servo_axes,
            test_mode=args.servo_test_mode,
        )
        vision = (
            VisionReceiver(VisionConfig(**json.loads(args.vision_config.read_text())))
            if args.vision_config
            else None
        )
        wheels = wheels_dashboard.DashboardRuntime(
            *wheel_settings,
            driver=HiwonderController(wheel_settings[1]) if wheel_hardware else None,
            allow_motion=args.wheel_allow_motion,
            config_path=args.wheel_config.resolve(),
            vision=vision,
        )
        servos = servo_dashboard.PanTiltRuntime(
            servo_settings,
            hardware=servo_hardware,
            allow_motion=args.servo_allow_motion,
            axes=args.servo_axes,
            test_mode=args.servo_test_mode,
            config_path=args.servo_config,
        )
        server = make_server(ControlDashboard(wheels, servos), args.host, args.port)
        if vision is not None:
            try:
                vision.start()
            except OSError:
                server.server_close()
                raise
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))

    workers = [
        threading.Thread(target=wheels.run, name="odometry-control", daemon=True),
        threading.Thread(target=servos.run, name="pan-tilt-control", daemon=True),
    ]
    previous = {}

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    try:
        for sig in (signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, interrupt)
        for worker in workers:
            worker.start()
        print(f"Main dashboard: http://{args.host}:{args.port}", flush=True)
        print(
            f"Wheels: {args.wheel_backend}, hardware motion: {wheels.allow_motion}; "
            f"pan/tilt: {args.servo_backend}, motion: {servos.allow_motion}, "
            f"axes: {', '.join(servos.axes)}, test mode: {servos.test_mode}",
            flush=True,
        )
        print(
            "Wheels start in Simulation. Switching tabs requests stop on the view left.", flush=True
        )
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        wheels.quit.set()
        servos.quit.set()
        for worker in workers:
            if worker.ident is not None:
                worker.join(timeout=5)
        server.server_close()
        if vision is not None:
            vision.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if any(worker.is_alive() for worker in workers):
            print("A control worker did not finish; stop could not be confirmed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
