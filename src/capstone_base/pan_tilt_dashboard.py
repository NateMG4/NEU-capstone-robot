"""Local servo test bench for one or both axes. Only the worker touches the bus."""

import argparse
import json
import math
import os
import secrets
import signal
import tempfile
import threading
import time
from collections import deque
from contextlib import ExitStack
from dataclasses import asdict, dataclass, replace
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from .dashboard_http import DashboardHandler
from .hiwonder_serial import LX224Bus
from .servo import FakeServoController, ServoConfig, _number, validate_feedback, validate_status

AXES = ("pan", "tilt")
HEARTBEAT_TIMEOUT_S = 2.5
TEST_JOG_MAX_DEG = 2.0
WEB = Path(__file__).with_name("pan_tilt_web")


@dataclass(frozen=True)
class AxisConfig:
    servo: ServoConfig
    zero_deg: float | None = 120
    direction: int = 1

    def __post_init__(self):
        if self.zero_deg is not None:
            _number(self.zero_deg, 0, 240, "zero_deg")
        if type(self.direction) is not int or self.direction not in (-1, 1):
            raise ValueError("direction must be 1 or -1")
        if self.servo.min_angle_deg is not None and self.zero_deg is not None:
            self.servo.validate_move(self.zero_deg, self.servo.min_duration_s)

    def joint(self, raw):
        return (raw - self.zero_deg) * self.direction if self.zero_deg is not None else None

    def raw(self, joint):
        if self.zero_deg is None:
            raise ValueError("Neutral is not calibrated; use a relative jog in --test-mode")
        _number(joint, -240, 240, "joint angle")
        return self.zero_deg + self.direction * joint

    def report(self):
        limits = None
        if self.servo.min_angle_deg is not None and self.zero_deg is not None:
            limits = sorted(map(self.joint, (self.servo.min_angle_deg, self.servo.max_angle_deg)))
        return {
            **asdict(self.servo),
            "zero_deg": self.zero_deg,
            "direction": self.direction,
            "joint_limits_deg": limits,
        }


def selected_axes(axes):
    if axes not in (*AXES, "both"):
        raise ValueError("axes must be pan, tilt, or both")
    return AXES if axes == "both" else (axes,)


def load_settings(path=None, *, hardware=False, axes="both", test_mode=False):
    enabled = selected_axes(axes)
    if path is None:
        if hardware:
            raise ValueError("Hardware requires --config with your measured pan/tilt settings")
        return {
            name: AxisConfig(ServoConfig(servo_id=i + 1, min_angle_deg=75, max_angle_deg=165))
            for i, name in enumerate(AXES)
        }
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {"port", "pan", "tilt"}:
        raise ValueError("Configuration must contain port, pan, and tilt")
    settings = {}
    for name in AXES:
        axis = dict(data[name])
        zero = axis.pop("zero_deg", 120)
        direction = axis.pop("direction", 1)
        if test_mode:
            zero = None
            axis.update(min_angle_deg=None, max_angle_deg=None)
        settings[name] = AxisConfig(ServoConfig(port=data["port"], **axis), zero, direction)
    if len(enabled) == 2 and settings["pan"].servo.servo_id == settings["tilt"].servo.servo_id:
        raise ValueError("Pan and tilt must have distinct servo IDs")
    if hardware and settings["pan"].servo.port is None:
        raise ValueError("Set port to the actual BusLinker serial device")
    return settings


class PanTiltRuntime:
    def __init__(
        self,
        settings,
        *,
        hardware=False,
        allow_motion=False,
        axes="both",
        test_mode=False,
        config_path=None,
        clock=time.monotonic,
    ):
        self.test_mode = test_mode
        self.launch_settings = dict(settings)
        self.config_path = Path(config_path).resolve() if config_path else None
        self.calibration_points = {
            name: {"direction": axis.direction} for name, axis in settings.items()
        }
        self.calibration_saved = set()
        self.settings = settings = {
            name: replace(
                axis,
                zero_deg=None,
                servo=replace(
                    axis.servo,
                    min_angle_deg=0,
                    max_angle_deg=240,
                    max_step_deg=min(TEST_JOG_MAX_DEG, axis.servo.max_step_deg),
                ),
            )
            if test_mode
            else axis
            for name, axis in settings.items()
        }
        self.axes = selected_axes(axes)
        if len({settings[name].servo.servo_id for name in self.axes}) != len(self.axes):
            raise ValueError("Enabled axes must have distinct servo IDs")
        self.clock = clock
        self.jog_max_deg = (
            min(TEST_JOG_MAX_DEG, *(settings[n].servo.max_step_deg for n in self.axes))
            if test_mode
            else 5
        )
        self.hardware = hardware
        self.allow_motion = not hardware or allow_motion
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.controllers = {}
        self.feedback = {name: None for name in AXES}
        self.sample_times = {name: None for name in AXES}
        self.errors = {name: None for name in AXES}
        self.pending = None
        self.active = False
        self.owner = None
        self.heartbeat_at = 0
        self.goals = {}
        self.step_targets = {}
        self.deadline = 0
        self.next_read = 0
        self.duration = 1
        self.legs = 0
        self.fault = None
        self.fault_report = None
        self.message = f"Connecting to {' and '.join(self.axes)}…"
        self.events = deque(maxlen=40)
        self.actuated = False

    def log(self, message):
        self.message = message
        self.events.appendleft({"time": time.strftime("%H:%M:%S"), "message": message})

    def _calibration_candidate(self, name, direction=None):
        points = self.calibration_points[name]
        if any(key not in points for key in ("zero_deg", "end_a", "end_b")):
            raise ValueError("Record neutral and both travel ends first")
        low, high = sorted((points["end_a"], points["end_b"]))
        base = self.launch_settings[name]
        return AxisConfig(
            replace(base.servo, min_angle_deg=low, max_angle_deg=high),
            points["zero_deg"],
            points["direction"] if direction is None else direction,
        )

    def _calibration_report(self, name):
        try:
            candidate = self._calibration_candidate(name).report()
            error = None
        except ValueError as exc:
            candidate, error = None, str(exc)
        return {
            "points": dict(self.calibration_points[name]),
            "candidate": candidate,
            "error": error,
            "saved": name in self.calibration_saved,
        }

    def snapshot(self):
        with self.lock:
            now = self.clock()
            axes = {}
            for name in AXES:
                status = self.feedback[name]
                config = self.settings[name]
                axes[name] = {
                    "enabled": name in self.axes,
                    "config": config.report(),
                    "status": asdict(status) if status else None,
                    "joint_deg": config.joint(status.position_deg) if status else None,
                    "age_s": now - self.sample_times[name] if status else None,
                    "error": self.errors[name],
                    "target_deg": config.joint(self.goals[name]) if name in self.goals else None,
                    "target_position_deg": self.goals.get(name),
                }
            return {
                "backend": "buslinker" if self.hardware else "simulation",
                "allow_motion": self.allow_motion,
                "test_mode": self.test_mode,
                "jog_max_deg": self.jog_max_deg,
                "calibration": {name: self._calibration_report(name) for name in self.axes},
                "calibration_file": str(self.config_path) if self.config_path else None,
                "enabled_axes": list(self.axes),
                "available": all(name in self.controllers for name in self.axes),
                "connected": (
                    all(name in self.controllers for name in self.axes)
                    and all(self.feedback[name] for name in self.axes)
                    and not any(self.errors[name] for name in self.axes)
                ),
                "active": self.active,
                "pending": self.pending is not None,
                "owner": self.owner,
                "fault": self.fault,
                "fault_report": self.fault_report,
                "message": self.message,
                "axes": axes,
                "events": list(self.events),
                "control_token": self.token,
                "heartbeat_timeout_s": HEARTBEAT_TIMEOUT_S,
            }

    def command(self, payload, now):
        """Validate and queue intent; HTTP threads never perform actuator I/O."""
        if not isinstance(payload, dict):
            raise ValueError("Expected a JSON object")
        action = payload.get("action")
        client = payload.get("client_id")
        if not isinstance(client, str) or not 1 <= len(client) <= 128:
            raise ValueError("A client_id is required")
        with self.lock:
            if action == "heartbeat":
                if client == self.owner:
                    self.heartbeat_at = now
                return
            if action == "stop":
                self.pending = {"action": "stop"}
                return
            axis = payload.get("axis", "both")
            if axis not in (*AXES, "both"):
                raise ValueError("axis must be pan, tilt, or both")
            selected = self.axes if axis == "both" else (axis,)
            if any(name not in self.axes for name in selected):
                raise ValueError(f"{axis} is disabled; restart with --axes {axis} to use it")
            if action in ("capture_calibration", "save_calibration", "use_calibration"):
                if self.active or self.pending:
                    raise ValueError("Stop or wait before recording calibration")
                if not all(name in self.controllers for name in self.axes):
                    raise ValueError("Servo connection is unavailable")
                command = {"action": action, "axis": axis}
                if action != "use_calibration" and axis == "both":
                    raise ValueError("Choose one axis to calibrate")
                if action == "capture_calibration":
                    point = payload.get("point")
                    if point not in ("zero_deg", "end_a", "end_b"):
                        raise ValueError("Choose neutral, end A, or end B")
                    command["point"] = point
                elif action == "save_calibration":
                    if not self.config_path:
                        raise ValueError("Start with --config to save calibration")
                    direction = payload.get("direction", 1)
                    self._calibration_candidate(axis, direction)
                    command["direction"] = direction
                elif not all(name in self.calibration_saved for name in self.axes):
                    raise ValueError("Save calibration for every enabled axis first")
                elif self.fault:
                    raise ValueError("Read status / clear fault before applying calibration")
                self.pending = command
                return
            if action == "torque" and payload.get("enabled") is False:
                # Release and stop are available to every browser, even after a fault.
                self.pending = {"action": "torque", "axis": axis, "enabled": False}
                return
            if action not in ("move", "jog", "neutral", "torque", "recover"):
                raise ValueError("Unknown action")
            if self.test_mode and action in ("move", "neutral"):
                raise ValueError(
                    "Test mode supports relative jogs; "
                    "neutral and absolute targets need calibration"
                )
            if self.active or self.pending:
                raise ValueError("Stop or wait for the current command to finish")
            if not all(name in self.controllers for name in self.axes):
                raise ValueError("Servo connection is unavailable; check the server terminal")
            if action != "recover":
                if self.fault:
                    raise ValueError("Read status / clear fault before issuing another command")
                if not self.allow_motion:
                    raise ValueError("Restart with --allow-motion to move or enable torque")
            command = {"action": action, "axis": axis}
            if action == "torque":
                if payload.get("enabled") is not True:
                    raise ValueError("enabled must be boolean")
                for name in selected:
                    self.settings[name].servo.require_limits()
                command["enabled"] = True
            if action in ("move", "jog", "neutral"):
                duration = payload.get("duration_s", 1)
                for name in selected:
                    config = self.settings[name]
                    _number(
                        duration,
                        config.servo.min_duration_s,
                        config.servo.max_duration_s,
                        "duration_s",
                    )
                command["duration_s"] = duration
                if action == "move":
                    fields = {"pan": "yaw_deg", "tilt": "pitch_deg"}
                    targets = {name: payload.get(fields[name]) for name in selected}
                elif action == "neutral":
                    targets = dict.fromkeys(selected, 0)
                else:
                    if axis == "both":
                        raise ValueError("Choose one axis to jog")
                    delta = payload.get("delta_deg")
                    _number(delta, -self.jog_max_deg, self.jog_max_deg, "jog delta_deg")
                    command["delta_deg"] = delta
                    targets = {}  # Resolve relative to fresh feedback in the worker.
                for name, joint in targets.items():
                    config = self.settings[name]
                    config.servo.validate_move(config.raw(joint), duration)
                command["targets"] = targets
                self.owner = client
                self.heartbeat_at = now
            self.pending = command

    def _read(self):
        failures = []
        for name, controller in self.controllers.items():
            try:
                self.feedback[name] = controller.read_status()
                self.sample_times[name] = self.clock()
                self.errors[name] = None
            except (OSError, ValueError, RuntimeError) as error:
                self.errors[name] = str(error)
                failures.append(f"{name}: {error}")
        if failures:
            raise OSError("; ".join(failures))

    def _health(self, *, torque=False, names=None):
        for name in self.axes if names is None else names:
            validate_status(self.feedback[name], self.settings[name].servo, require_torque=torque)

    def _stop(self):
        self.active = False
        self.owner = None
        self.step_targets = {}
        self.goals = {}
        failures = []
        for name, controller in self.controllers.items():
            try:
                controller.stop()
            except (OSError, RuntimeError) as error:
                failures.append(f"{name}: {error}")
        if failures:
            raise OSError("Stop unconfirmed: " + "; ".join(failures))
        self.actuated = False

    def _trip(self, error):
        report = {
            "time": time.strftime("%H:%M:%S"),
            "duration_s": self.duration,
            "axes": {
                name: {
                    "goal_position_deg": self.goals.get(name),
                    "step_target_position_deg": self.step_targets.get(name),
                    "feedback_position_deg": (
                        self.feedback[name].position_deg if self.feedback[name] else None
                    ),
                    "tolerance_deg": self.settings[name].servo.position_tolerance_deg,
                }
                for name in self.axes
            },
        }
        was_active = self.active or self.actuated
        self.active = False
        self.owner = None
        self.pending = None
        if was_active:
            try:
                self._stop()
            except (OSError, RuntimeError) as stop_error:
                error = f"{error}; {stop_error}"
        self.fault = str(error)
        self.fault_report = {**report, "message": self.fault}
        self.log(self.fault)

    def _start_step(self, now):
        self.step_targets = {}
        # Initialization reads fresh feedback. Plan from those positions rather
        # than the earlier telemetry sample, which can shift while settling.
        for name in self.goals:
            self.feedback[name] = self.controllers[name].initialize()
            self.sample_times[name] = self.clock()
        self._health(torque=True)
        targets = {}
        for name, goal in self.goals.items():
            config = self.settings[name].servo
            current = self.feedback[name].position_deg
            # Leave room for settling between planning and the driver's final
            # position read. The driver still enforces the full configured cap.
            margin = min(config.position_tolerance_deg, config.max_step_deg / 2)
            step_ticks = max(1, math.floor((config.max_step_deg - margin) / 0.24 + 1e-9))
            delta = max(-step_ticks, min(step_ticks, round((goal - current) / 0.24)))
            target = round(current / 0.24 + delta) * 0.24
            config.validate_move(target, self.duration)
            if abs(target - current) > 0.12:
                targets[name] = target
        if not targets:
            self._stop()
            self.log("Target reached; holding position")
            return
        self.legs += 1
        if self.legs > 2048:
            raise ValueError("Too many movement steps; inspect position feedback")
        # Every selected axis has been preflighted before the first actuator write.
        if self.clock() - self.heartbeat_at > HEARTBEAT_TIMEOUT_S:
            raise RuntimeError("Browser heartbeat expired during servo checks; movement cancelled")
        self.step_targets = targets
        for name, target in targets.items():
            self.actuated = True
            self.controllers[name].move_to(target, self.duration)
        self.deadline = self.clock() + self.duration + 0.15

    def _execute(self, command, now):
        action = command["action"]
        if action in ("capture_calibration", "save_calibration", "use_calibration"):
            self._read()
            name = command["axis"]
            if action == "capture_calibration":
                position = self.feedback[name].position_deg
                _number(position, 0, 240, "calibration servo angle")
                self.calibration_points[name][command["point"]] = position
                self.calibration_saved.discard(name)
                self.log(f"Recorded {name} {command['point']}: {position:.2f}° servo angle")
            elif action == "save_calibration":
                candidate = self._calibration_candidate(name, command["direction"])
                data = json.loads(self.config_path.read_text())
                if data[name]["servo_id"] != candidate.servo.servo_id:
                    raise ValueError("Servo ID changed in the file; restart before saving")
                data[name].update(
                    zero_deg=candidate.zero_deg,
                    direction=candidate.direction,
                    min_angle_deg=candidate.servo.min_angle_deg,
                    max_angle_deg=candidate.servo.max_angle_deg,
                )
                temporary = None
                try:
                    with tempfile.NamedTemporaryFile(
                        mode="w", dir=self.config_path.parent, prefix=".calibration-", delete=False
                    ) as file:
                        temporary = Path(file.name)
                        json.dump(data, file, indent=2)
                        file.write("\n")
                    temporary.chmod(self.config_path.stat().st_mode & 0o777)
                    os.replace(temporary, self.config_path)
                finally:
                    if temporary is not None:
                        temporary.unlink(missing_ok=True)
                self.launch_settings[name] = candidate
                self.calibration_points[name]["direction"] = candidate.direction
                self.calibration_saved.add(name)
                self.log(f"Saved {name} calibration to {self.config_path.name}")
            else:
                for axis in self.axes:
                    validate_status(self.feedback[axis], self.launch_settings[axis].servo)
                for axis in self.axes:
                    self.settings[axis] = self.launch_settings[axis]
                    self.controllers[axis].config = self.settings[axis].servo
                self.test_mode = False
                self.jog_max_deg = 5
                self.log("Saved calibration applied; position preserved")
            return
        if action == "stop":
            self._stop()
            self.log(f"Stop requested for {' and '.join(self.axes)}; torque preserved")
            return
        if action == "torque":
            selected = self.axes if command["axis"] == "both" else (command["axis"],)
            self._stop()
            if command["enabled"]:
                self._read()
                self._health(names=selected)
            failures = []
            for name in selected:
                try:
                    self.actuated = True
                    self.controllers[name].set_torque_enabled(command["enabled"])
                except (OSError, ValueError, RuntimeError) as error:
                    failures.append(f"{name}: {error}")
            if failures:
                raise OSError("; ".join(failures))
            self._read()
            self.log(
                f"Torque {'enabled' if command['enabled'] else 'released'}: "
                + " and ".join(selected)
            )
            return
        self._read()
        if action == "recover":
            # A released joint can move beyond its travel limits. Recovery reads
            # health only; future actuator commands must pass their own checks.
            for name in self.axes:
                validate_feedback(self.feedback[name], self.settings[name].servo)
            self.fault = None
            self.log("Fault cleared; motion remains stopped. Enable torque explicitly if released.")
            return
        self._health(torque=True)
        targets = command["targets"]
        if action == "jog":
            name = command["axis"]
            if self.test_mode:
                targets = {
                    name: self.feedback[name].position_deg
                    + self.settings[name].direction * command["delta_deg"]
                }
            else:
                targets = {
                    name: self.settings[name].joint(self.feedback[name].position_deg)
                    + command["delta_deg"]
                }
        self.duration = command["duration_s"]
        self.goals = {}
        for name, joint in targets.items():
            config = self.settings[name]
            raw = joint if self.test_mode else config.raw(joint)
            position, _ = config.servo.validate_move(raw, self.duration)
            self.goals[name] = position * 0.24
        self.active = True
        self.legs = 0
        self.log(
            "Moving "
            + ", ".join(f"{name} to servo {goal:.2f}°" for name, goal in self.goals.items())
            + f" ({self.duration:g}s per step)"
        )
        self._start_step(now)

    def tick(self, now):
        with self.lock:
            try:
                safety_command = self.pending and (
                    self.pending["action"] == "stop"
                    or (self.pending["action"] == "torque" and not self.pending["enabled"])
                )
                if safety_command:
                    command, self.pending = self.pending, None
                    self._execute(command, now)
                elif self.owner and now - self.heartbeat_at > HEARTBEAT_TIMEOUT_S:
                    self.pending = None
                    self._stop()
                    self.log(f"Browser heartbeat lost; stopped {' and '.join(self.axes)}")
                    return
                elif self.pending:
                    command, self.pending = self.pending, None
                    self._execute(command, now)
                if self.fault:
                    # A movement fault stays latched, but feedback must remain
                    # readable so the operator can inspect torque and position.
                    if now >= self.next_read:
                        self.next_read = now + 0.5
                        try:
                            self._read()
                        except (OSError, ValueError, RuntimeError):
                            pass  # Per-axis errors are published; retain the original fault.
                    return
                if now >= self.next_read:
                    self._read()
                    self.next_read = now + 0.5
                    if self.active:
                        self._health(torque=True)
                if self.active and now >= self.deadline:
                    self._read()
                    for name, target in self.step_targets.items():
                        tolerance = self.settings[name].servo.position_tolerance_deg
                        if abs(self.feedback[name].position_deg - target) > tolerance:
                            actual = self.feedback[name].position_deg
                            raise ValueError(
                                f"{name}: target not reached; servo target {target:.2f}°, "
                                f"feedback {actual:.2f}°, error {abs(actual - target):.2f}° "
                                f"exceeds {tolerance:g}° after {self.duration:g}s. "
                                "Check clearance and try a longer step time."
                            )
                    reached = all(
                        abs(self.feedback[name].position_deg - goal)
                        <= self.settings[name].servo.position_tolerance_deg
                        and (
                            name not in self.step_targets
                            or abs(self.step_targets[name] - goal) < 0.12
                        )
                        for name, goal in self.goals.items()
                    )
                    if reached:
                        self._stop()
                        self.log("Target reached within tolerance; holding position")
                    else:
                        self._start_step(now)
            except (OSError, ValueError, RuntimeError) as error:
                self._trip(error)

    def run(self):
        try:
            with ExitStack() as stack:
                if self.hardware:
                    bus = stack.enter_context(LX224Bus(self.settings[self.axes[0]].servo))
                    controllers = {
                        name: bus.controller(self.settings[name].servo) for name in self.axes
                    }
                else:
                    controllers = {
                        name: FakeServoController(self.settings[name].servo) for name in self.axes
                    }
                with self.lock:
                    for name, controller in controllers.items():
                        self.controllers[name] = stack.enter_context(controller)
                    self.log(
                        f"Connected to {' and '.join(self.axes)}; waiting for an explicit command"
                    )
                try:
                    while not self.quit.is_set():
                        self.tick(time.monotonic())
                        self.quit.wait(0.04)
                finally:
                    with self.lock:
                        if self.active or self.actuated:
                            self._stop()
        except (OSError, ValueError, RuntimeError) as error:
            with self.lock:
                self._trip(error)
                self.controllers = {}


def make_handler(runtime):
    class Handler(DashboardHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/api/state":
                self.reply(200, runtime.snapshot())
                return
            assets = {
                "/": ("index.html", "text/html; charset=utf-8"),
                "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                "/controls.mjs": ("controls.mjs", "text/javascript; charset=utf-8"),
                "/style.css": ("style.css", "text/css; charset=utf-8"),
            }
            if path not in assets:
                self.reply(404, {"error": "Not found"})
                return
            filename, content_type = assets[path]
            self.reply(200, (WEB / filename).read_bytes(), content_type)

        def do_POST(self):
            self.connection.settimeout(2)
            if self.path != "/api/command":
                self.reply(404, {"error": "Not found"})
                return
            origin = self.headers.get("Origin")
            if (
                origin and urlsplit(origin).netloc != self.headers.get("Host")
            ) or not secrets.compare_digest(self.headers.get("X-Control-Token", ""), runtime.token):
                self.reply(403, {"error": "Send commands from this dashboard"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("Invalid request size")
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected application/json")
                runtime.command(json.loads(self.rfile.read(length)), time.monotonic())
            except (OSError, ValueError, TypeError, UnicodeError) as error:
                self.reply(400, {"error": str(error)})
                return
            self.reply(200, {"ok": True})

    return Handler


def make_server(runtime, host, port):
    return ThreadingHTTPServer((host, port), make_handler(runtime))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Pan/tilt test dashboard for one or both servos")
    parser.add_argument("--backend", choices=("simulation", "buslinker"), default="simulation")
    parser.add_argument("--config", type=Path)
    parser.add_argument(
        "--test-mode",
        action="store_true",
        help="Uncalibrated bring-up: relative jogs up to 2 degrees, no neutral",
    )
    parser.add_argument(
        "--axes", choices=("both", *AXES), default="both", help="Servos connected for this session"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument(
        "--allow-motion", action="store_true", help="Allow hardware moves / torque on"
    )
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.port <= 65535:
            raise ValueError("Port must be between 1 and 65535")
        hardware = args.backend == "buslinker"
        if args.allow_motion and not hardware:
            raise ValueError("--allow-motion requires --backend buslinker")
        settings = load_settings(
            args.config, hardware=hardware, axes=args.axes, test_mode=args.test_mode
        )
        runtime = PanTiltRuntime(
            settings,
            hardware=hardware,
            allow_motion=args.allow_motion,
            axes=args.axes,
            test_mode=args.test_mode,
            config_path=args.config,
        )
        server = make_server(runtime, args.host, args.port)
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))
    worker = threading.Thread(target=runtime.run, name="pan-tilt-control", daemon=True)
    worker.start()
    print(f"Pan/tilt dashboard: http://{args.host}:{args.port}", flush=True)
    print(f"Backend: {args.backend} | Motion allowed: {runtime.allow_motion}", flush=True)
    print(f"Enabled axes: {', '.join(runtime.axes)}", flush=True)
    print("Test mode: " + str(runtime.test_mode), flush=True)
    previous = {}

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    try:
        for sig in (signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, interrupt)
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        runtime.quit.set()
        worker.join(timeout=5)
        server.server_close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if worker.is_alive():
            print("Servo thread did not finish; stop could not be confirmed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
