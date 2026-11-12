"""Pi-local encoder/navigation dashboard. Defaults to a separate simulated plant."""

import argparse
import json
import math
import secrets
import signal
import threading
import time
from dataclasses import asdict, replace
from http.server import ThreadingHTTPServer
from importlib.resources import files
from pathlib import Path
from urllib.parse import urlsplit

from .dashboard_http import DashboardHandler
from .hiwonder_i2c import HiwonderConfig, HiwonderController, wheel_targets_to_counts
from .kinematics import Geometry
from .navigation import NavigationLimits, pose_dict, ramp_wheels, target_wheels
from .odometry import (
    EncoderOdometry,
    Pose,
    ReferenceFrame,
    counter_delta,
    finite,
    validate_counters,
    wrap,
)
from .vision import VisionConfig, VisionReceiver

ZERO = (0.0, 0.0, 0.0, 0.0)


def load_settings(path, hardware=False):
    data = json.loads(Path(path).read_text())
    if type(data.get("simulation_only")) is not bool:
        raise ValueError("Configuration needs simulation_only=true or false")
    if hardware and data["simulation_only"]:
        raise ValueError("Hardware requires measured simulation_only=false configuration")
    geometry = Geometry(data["wheel_radius_m"], data["wheelbase_m"], data["track_width_m"])
    config = HiwonderConfig(**data["hiwonder"])
    if any(
        v is None
        for v in (
            config.encoder_signs,
            config.counts_per_wheel_rev,
            config.motor_signs,
            config.wheel_channels,
        )
    ):
        raise ValueError(
            "Dashboard needs measured wheel channels, both sign maps and encoder scales"
        )
    navigation = dict(data.get("navigation", {}))
    navigation.setdefault("max_wheel_rad_s", data["max_wheel_rad_s"])
    limits = NavigationLimits(**navigation)
    count_cap = min(
        config.max_speed_counts * math.tau / (n * 0.01) for n in config.counts_per_wheel_rev
    )
    # Leave numerical headroom for rad/s -> count round trips at the exact cap.
    limits = replace(limits, max_wheel_rad_s=min(limits.max_wheel_rad_s, count_cap * (1 - 1e-12)))
    # Preflight hardware conversion, including all calibration fields, before bus access.
    wheel_targets_to_counts(ZERO, config)
    return geometry, config, limits


class SimulatedEncoders:
    """Ideal quantized wheel plant; no traction, inertia, or independent ground truth."""

    def __init__(self, config):
        self.config = config
        self.positions = [0.0] * 4  # Raw M1..M4 counts, retained with fractional precision.
        self.speeds = ZERO

    def advance(self, dt):
        for i, value in enumerate(self.speeds):
            self.positions[i] += value * 100 * dt

    def read_encoder_counts(self):
        return tuple(((round(n) + 2**31) % 2**32) - 2**31 for n in self.positions)

    def set_wheel_speeds(self, targets):
        # Model this board's integer count commands, not a perfect unquantized pose.
        self.speeds = wheel_targets_to_counts(targets, self.config)

    def stop(self):
        self.speeds = ZERO


class DashboardRuntime:
    """Single control-thread owner of I2C; HTTP handlers only change intent under a lock."""

    def __init__(
        self,
        geometry,
        config,
        limits,
        *,
        driver=None,
        allow_motion=False,
        config_path=None,
        vision=None,
    ):
        self.geometry, self.config, self.limits = geometry, config, limits
        self.launch_limits = limits
        k = (geometry.wheelbase_m + geometry.track_width_m) / 2
        # Tuning can vary desired chassis motion, but cannot raise the wheel
        # ceiling established by configuration and the motor count caps.
        ranges = {
            "max_speed_m_s": (0.001, geometry.wheel_radius_m * limits.max_wheel_rad_s),
            "max_yaw_rad_s": (0.01, geometry.wheel_radius_m * limits.max_wheel_rad_s / k),
            "max_wheel_rad_s": (0.01, limits.max_wheel_rad_s),
            "wheel_accel_rad_s2": (0.1, 12.0),
            "position_gain": (0.05, 3.0),
            "yaw_gain": (0.05, 3.0),
            "position_tolerance_m": (0.005, 0.25),
            "yaw_tolerance_rad": (math.radians(0.5), math.radians(30)),
        }
        self.navigation_bounds = {
            name: {"min": min(low, getattr(limits, name)), "max": max(high, getattr(limits, name))}
            for name, (low, high) in ranges.items()
        }
        self.driver = driver
        self.config_path = str(config_path) if config_path is not None else None
        self.hardware_open = False
        self.control_online = True
        self.recovery_requested = False
        self.recovering = False
        self.recovery_count = 0
        self.fault_report = None
        self.sample_source = None
        self.latest_reads = {mode: None for mode in ("simulation", "hardware")}
        self.pose_uncertain = {mode: False for mode in ("simulation", "hardware")}
        self.allow_motion = bool(allow_motion and driver is not None)
        self.sim = SimulatedEncoders(config)
        self.odometry = {
            mode: EncoderOdometry(
                geometry, config, max_wheel_rad_s=max(20, limits.max_wheel_rad_s * 4)
            )
            for mode in ("simulation", "hardware")
        }
        self.frames = {mode: ReferenceFrame() for mode in self.odometry}
        self.vision = vision
        self.camera_enabled = {mode: False for mode in self.odometry}
        self.camera_applied = {mode: None for mode in self.odometry}
        self.camera_error = {mode: None for mode in self.odometry}
        self.camera_align_requested = None
        self.trails = {mode: [] for mode in self.odometry}
        self.mode = "simulation"
        self.target = self.start = Pose()
        self.active = False
        self.owner = None
        self.lease_until = self.started = self.settled_since = None
        self.last_tick = None
        self.applied = ZERO
        self.hardware_sent = ZERO
        self.fault = None
        self.status = "Preview — select a target"
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.token = secrets.token_urlsafe(24)

    def pose(self):
        return self.frames[self.mode].transform(self.odometry[self.mode].pose)

    def disarm(self, message):
        self.active = False
        self.applied = ZERO
        self.settled_since = None
        self.status = message

    def command(self, action, payload, client, now):
        finite(now, "command time")
        if not isinstance(client, str) or not 1 <= len(client) <= 100:
            raise ValueError("A client_id is required")
        with self.lock:
            if action == "stop":  # Any tab can stop; only the controlling tab can renew motion.
                self.disarm("Stopped")
                return
            if self.active and now >= self.lease_until:
                self.disarm("Browser heartbeat expired — stopped")
            if self.active and self.owner != client:
                raise ValueError("Another browser controls motion; stop it before taking control")
            if action == "heartbeat":
                if self.active:
                    self.lease_until = now + 0.75
                return
            if action == "recover":
                if not self.fault:
                    raise ValueError("There is no fault to recover")
                if not self.control_online:
                    raise ValueError("Control worker is stopped; restart the server")
                if self.recovering or self.recovery_requested:
                    raise ValueError("Recovery is already in progress")
                self.disarm("Recovery queued — movement remains off")
                self.owner = None
                self.recovery_requested = True
                return
            if self.fault:
                raise ValueError(
                    "Control fault is latched; use Recover encoders after checking diagnostics"
                )
            if action == "mode":
                mode = payload.get("mode")
                if mode not in self.odometry or (mode == "hardware" and self.driver is None):
                    raise ValueError("Live encoders require launching with --backend hiwonder")
                self.disarm("Mode changed — preview only")
                self.camera_align_requested = None
                self.mode = mode
                self.target = self.start = self.pose()
            elif action == "target":
                x, y, heading = (finite(payload.get(n), n) for n in ("x", "y", "heading_deg"))
                if not -180 <= heading <= 180:
                    raise ValueError("Heading must be between -180 and 180 degrees")
                current = self.pose()
                if math.hypot(x - current.x, y - current.y) > 2.0:
                    raise ValueError("Select a target within 2 m of the current estimated position")
                self.disarm("Target ready — preview only")
                self.target, self.start = Pose(x, y, math.radians(heading)), current
            elif action == "apply":
                if self.active:
                    raise ValueError("Target is already active")
                if self.mode == "hardware" and not self.allow_motion:
                    raise ValueError(
                        "Hardware motion disabled; restart with --allow-motion when calibrated"
                    )
                if self.odometry[self.mode].time is None:
                    raise ValueError("Waiting for an encoder baseline")
                if self.pose_uncertain[self.mode]:
                    raise ValueError(
                        "Position is uncertain; reset the local origin before applying a target"
                    )
                self.active, self.owner = True, client
                self.lease_until, self.started = now + 0.75, now
                self.settled_since = None
                self.status = "Applying target" if self.mode == "hardware" else "Running simulation"
            elif action == "reset":
                self.disarm("New local origin — preview only")
                self.odometry[self.mode].reset()
                self.pose_uncertain[self.mode] = False
                self.frames[self.mode] = ReferenceFrame()
                self.camera_enabled[self.mode] = False
                self.camera_applied[self.mode] = None
                self.camera_error[self.mode] = None
                self.camera_align_requested = None
                self.trails[self.mode].clear()
                self.target = self.start = Pose()
            elif action == "camera_align":
                if self.vision is None or self.vision.observation(now) is None:
                    raise ValueError(
                        "Waiting for a fresh camera pose; check robot ID and clock sync"
                    )
                self.disarm("Camera alignment queued — movement off")
                self.camera_enabled[self.mode] = False
                self.camera_align_requested = self.mode
            elif action == "camera_disable":
                self.camera_align_requested = None
                self.camera_enabled[self.mode] = False
                self.disarm("Camera corrections off — encoder estimate in the current frame")
            elif action in ("navigation", "navigation_reset"):
                if action == "navigation_reset":
                    updated = self.launch_limits
                else:
                    values = payload.get("navigation")
                    if not isinstance(values, dict) or not values:
                        raise ValueError("Supply navigation settings as a nonempty object")
                    normalized = {}
                    for name, value in values.items():
                        if name not in self.navigation_bounds:
                            raise ValueError(f"Unknown navigation setting: {name}")
                        value = finite(value, name)
                        bounds = self.navigation_bounds[name]
                        if (
                            value < bounds["min"]
                            and not math.isclose(value, bounds["min"], rel_tol=1e-10)
                        ) or (
                            value > bounds["max"]
                            and not math.isclose(value, bounds["max"], rel_tol=1e-10)
                        ):
                            raise ValueError(
                                f"{name} must be between {bounds['min']:g} and {bounds['max']:g}"
                            )
                        # Tolerate display rounding, while retaining exact server ceilings.
                        normalized[name] = min(bounds["max"], max(bounds["min"], value))
                    updated = replace(self.limits, **normalized)
                self.disarm("Navigation updated — preview only; apply target to resume")
                self.limits = updated
            else:
                raise ValueError("Unknown action")

    def observe_pose(self, absolute_pose, timestamp, frame_id, now):
        """Align an absolute pose using this process's monotonic capture timestamp.

        Align against historical odometry, preserving motion since capture. This
        intentionally stops/clears goals when changing coordinates. Continuous
        same-frame camera updates use update_camera() with discrepancy gates.
        """
        finite(now, "receive time")
        finite(timestamp, "observation time")
        with self.lock:
            if not 0 <= now - timestamp <= 0.5:
                raise ValueError("Absolute observation is stale or from a different clock")
            odo = self.odometry[self.mode]
            captured = odo.pose_at(timestamp)
            self.frames[self.mode].align(absolute_pose, captured, frame_id)
            self.pose_uncertain[self.mode] = False
            self.disarm("Absolute frame aligned — reselect target")
            self.target = self.start = self.pose()

    def update_camera(self, now):
        """Control-thread pose correction. Dropouts retain the last frame alignment."""
        if self.vision is None:
            return
        obs = self.vision.observation(now)
        mode = self.mode
        requested = self.camera_align_requested == mode
        if not requested and not self.camera_enabled[mode]:
            return
        if obs is None:
            if requested:
                self.camera_align_requested = None
                self.camera_error[mode] = "Camera alignment expired; request a fresh alignment"
                self.status = "Camera alignment expired — movement remains off"
            return  # Encoder fallback, with no coordinate-frame or target change.
        if not requested and self.camera_applied[mode] == obs.epoch:
            return
        try:
            captured = self.odometry[mode].pose_at(obs.timestamp)
            cfg = self.vision.config
            if requested:
                self.observe_pose(obs.pose, obs.timestamp, cfg.frame_id, now)
                self.camera_enabled[mode] = True
            else:
                expected = self.frames[mode].transform(captured)
                distance = math.hypot(obs.pose.x - expected.x, obs.pose.y - expected.y)
                heading = abs(wrap(obs.pose.yaw - expected.yaw))
                if distance > cfg.max_position_jump_m or heading > math.radians(
                    cfg.max_heading_jump_deg
                ):
                    self.camera_enabled[mode] = False
                    self.disarm("Camera disagrees with encoders — inspect and realign")
                    raise ValueError(
                        f"Camera discrepancy: {distance:.3f} m / {math.degrees(heading):.1f} deg; "
                        "inspect calibration/identity, then explicitly align again"
                    )
                self.frames[mode].align(obs.pose, captured, cfg.frame_id)
            self.camera_applied[mode] = obs.epoch
            self.camera_error[mode] = None
        except ValueError as error:
            # A packet outside a new/reset encoder history cannot bridge a fault.
            self.camera_error[mode] = str(error)
            if requested:
                self.status = "Camera alignment failed — wait for a new pose and align again"
        finally:
            if requested:
                self.camera_align_requested = None

    def tick(self, now, *, clock=None):
        with self.lock:
            if self.fault or self.recovering:
                return  # Only an explicit queued recovery can replace a rejected baseline.
            self.sample_source = None
            dt = 0.05 if self.last_tick is None else now - self.last_tick
            self.last_tick = now
            if not 0 < dt <= 0.5:
                raise ValueError("Control loop stalled; restart and rebaseline")
            self.sim.advance(dt)
            self.update_encoders("simulation", self.sim.read_encoder_counts(), now)
            if self.driver is not None:
                self.sample_source = "hardware"
                raw = self.driver.read_encoder_counts()
                sampled = now if clock is None else clock()
                self.update_encoders("hardware", raw, sampled)
            # Expire against fresh time after I/O, rather than the loop's start:
            # a slow board read must not extend a browser's movement lease.
            checked = now if clock is None else clock()
            if checked - now > 0.5:
                raise ValueError("Control I/O stalled; restart and rebaseline")
            now = checked
            self.update_camera(now)
            for mode, odo in self.odometry.items():
                if odo.time is not None:
                    trail = self.trails[mode]
                    if (
                        not trail
                        or math.hypot(odo.pose.x - trail[-1][0], odo.pose.y - trail[-1][1]) >= 0.003
                    ):
                        trail.append((odo.pose.x, odo.pose.y))
                        del trail[:-1200]
            if self.active and now >= self.lease_until:
                self.disarm("Browser heartbeat expired — stopped")
            if self.active and now - self.started >= 30:
                self.disarm("30-second move limit reached — stopped")
            plan = target_wheels(self.pose(), self.target, self.geometry, self.limits)
            if self.active and plan["reached"]:
                moving = max(abs(v) for v in self.odometry[self.mode].wheel_rad_s) > 0.15
                if moving:
                    self.settled_since = None
                elif self.settled_since is None:
                    self.settled_since = now
                elif now - self.settled_since >= 0.3:
                    source = (
                        "camera-corrected estimate"
                        if self.camera_enabled[self.mode]
                        and self.vision.observation(now) is not None
                        and self.camera_error[self.mode] is None
                        else "encoder estimate"
                    )
                    self.disarm(f"Target reached ({source})")
            else:
                self.settled_since = None
            self.applied = (
                ramp_wheels(self.applied, plan["wheels"], dt, self.limits.wheel_accel_rad_s2)
                if self.active
                else ZERO
            )
            self.sim.set_wheel_speeds(self.applied if self.mode == "simulation" else ZERO)
            hardware = self.applied if self.mode == "hardware" and self.active else ZERO
            if self.driver is not None:
                if hardware != ZERO:
                    self.driver.set_wheel_speeds(hardware)
                elif self.hardware_sent != ZERO:
                    self.driver.stop()
                self.hardware_sent = hardware

    def update_encoders(self, source, raw, now):
        self.sample_source = source
        self.latest_reads[source] = {"raw_counts": tuple(raw), "time": now}
        return self.odometry[source].update(raw, now)

    def configuration_report(self):
        return {
            "path": self.config_path,
            "counts_per_wheel_rev": self.config.counts_per_wheel_rev,
            "wheel_channels": self.config.wheel_channels,
            "encoder_signs": self.config.encoder_signs,
            "motor_signs": self.config.motor_signs,
            "motor_type": self.config.motor_type,
            "encoder_polarity": self.config.encoder_polarity,
            "max_speed_counts": self.config.max_speed_counts,
            "max_encoder_wheel_rad_s": self.odometry["hardware"].max_wheel_rad_s,
        }

    def trip(self, error, *, retain_report=False):
        with self.lock:
            self.camera_align_requested = None
            for mode in self.camera_enabled:
                self.camera_enabled[mode] = False
            self.fault = str(error)
            source = self.sample_source or self.mode
            if not retain_report:
                self.pose_uncertain[source] = True
                details = getattr(error, "details", {})
                wheel_command = (
                    self.hardware_sent
                    if source == "hardware"
                    else (self.applied if self.mode == "simulation" else ZERO)
                )
                try:
                    motor_counts = wheel_targets_to_counts(wheel_command, self.config)
                except (ValueError, TypeError):
                    motor_counts = None
                dt = details.get("dt_s", 0)
                expected = (
                    tuple(abs(n) * 100 * dt for n in motor_counts)
                    if motor_counts is not None and dt > 0
                    else None
                )
                self.fault_report = {
                    "message": self.fault,
                    "source": source,
                    "time": time.monotonic(),
                    "details": details,
                    "last_accepted_raw": self.odometry[source].raw,
                    "latest_read": self.latest_reads[source],
                    "requested_wheel_rad_s": wheel_command,
                    "motor_command_counts_per_10ms": motor_counts,
                    "expected_abs_delta_from_command": expected,
                    "configuration": self.configuration_report(),
                }
                print(
                    "Dashboard fault: " + json.dumps(self.fault_report, allow_nan=False), flush=True
                )
            self.disarm("Fault — motion disabled")
            self.sim.stop()
            if self.driver is not None:
                try:
                    self.driver.stop()
                except (OSError, RuntimeError) as stop_error:
                    self.fault += f"; STOP NOT CONFIRMED: {stop_error}"

    def recover(self, *, clock=time.monotonic, wait=None):
        """Control-thread-only recovery. Never called directly by an HTTP handler."""
        wait = self.quit.wait if wait is None else wait
        with self.lock:
            if not self.recovery_requested or self.recovering:
                return
            self.recovery_requested = False
            self.recovering = True
            self.disarm("Recovering — confirming stopped encoder readings")
            self.owner = None
            self.sim.stop()
        try:
            if self.driver is not None:
                # All hardware I/O remains in the control worker. Initialization
                # re-arms the driver's communication state, but commands zero only.
                if not self.hardware_open:
                    self.driver.open()
                    self.hardware_open = True
                self.driver.stop()
                self.driver.initialize()
                previous = validate_counters(self.driver.read_encoder_counts())
                stable = 0
                for _ in range(20):
                    if wait(0.05) or self.quit.is_set():
                        raise RuntimeError("Recovery cancelled during server shutdown")
                    current = validate_counters(self.driver.read_encoder_counts())
                    if all(
                        abs(counter_delta(a, b)) <= 2
                        for a, b in zip(current, previous, strict=True)
                    ):
                        stable += 1
                    else:
                        stable = 0
                    previous = current
                    with self.lock:
                        self.latest_reads["hardware"] = {"raw_counts": current, "time": clock()}
                    if stable >= 3:
                        break
                else:
                    raise ValueError(
                        "Encoders did not settle; check movement, wiring and raw reads"
                    )
            now = clock()
            if self.quit.is_set():
                raise RuntimeError("Recovery cancelled during server shutdown")
            with self.lock:
                self.odometry["simulation"].rebaseline(self.sim.read_encoder_counts(), now)
                if self.driver is not None:
                    self.odometry["hardware"].rebaseline(current, now)
                for trail in self.trails.values():
                    trail.clear()  # Do not join paths across discarded, uncertain motion.
                self.last_tick = now
                self.hardware_sent = ZERO
                self.fault = None
                self.recovery_count += 1
                self.target = self.start = self.pose()
                self.disarm("Recovered — pose held at last valid estimate; reselect target")
        except (OSError, RuntimeError, ValueError, TypeError) as error:
            self.trip(RuntimeError(f"Recovery failed: {error}"), retain_report=True)
        finally:
            with self.lock:
                self.recovering = False

    def run(self):
        try:
            if self.driver is not None:
                try:
                    self.sample_source = "hardware"
                    self.driver.open()
                    self.hardware_open = True
                    self.driver.initialize()
                except (OSError, RuntimeError, ValueError) as error:
                    self.trip(error)
            while not self.quit.is_set():
                start = time.monotonic()
                try:
                    if self.recovery_requested:
                        self.recover()
                        self.quit.wait(0.05)  # Start the new baseline with a full sample interval.
                        continue
                    elif not self.fault:
                        self.tick(start, clock=time.monotonic)
                except (OSError, RuntimeError, ValueError, TypeError) as error:
                    self.trip(error)
                self.quit.wait(max(0, 0.05 - (time.monotonic() - start)))
        except (OSError, RuntimeError, ValueError) as error:
            self.trip(error)
        finally:
            self.control_online = False
            if self.driver is not None:
                try:
                    self.driver.close()
                except OSError as error:
                    self.trip(error)

    def snapshot(self):
        with self.lock:
            odo = self.odometry[self.mode]
            plan = target_wheels(self.pose(), self.target, self.geometry, self.limits)
            frame = self.frames[self.mode]
            trail = [pose_dict(frame.transform(Pose(x, y))) for x, y in self.trails[self.mode]]
            camera = (
                self.vision.snapshot(time.monotonic())
                if self.vision is not None
                else {"configured": False, "connected": False}
            )
            camera.update(
                {
                    "enabled": self.camera_enabled[self.mode],
                    "alignment_pending": self.camera_align_requested == self.mode,
                    "correction_error": self.camera_error[self.mode],
                    "last_applied_epoch": self.camera_applied[self.mode],
                }
            )
            return {
                "control_token": self.token,
                "mode": self.mode,
                "hardware_available": self.driver is not None,
                "hardware_motion_allowed": self.allow_motion,
                "active": self.active,
                "status": self.status,
                "fault": self.fault,
                "fault_report": self.fault_report,
                "recovery_pending": self.recovery_requested or self.recovering,
                "recovery_available": bool(self.fault) and self.control_online,
                "recovery_count": self.recovery_count,
                "pose_uncertain": self.pose_uncertain[self.mode],
                "encoder_configuration": self.configuration_report(),
                "latest_encoder_read": self.latest_reads[self.mode],
                "pose": pose_dict(self.pose()),
                "target": pose_dict(self.target),
                "path": [pose_dict(self.start), pose_dict(self.target)],
                "trail": trail,
                "frame_id": frame.frame_id,
                "camera_connected": camera["connected"],
                "camera": camera,
                "encoder_pose": pose_dict(odo.pose),
                "raw_counts": odo.raw,
                "delta_counts": odo.delta_counts,
                "wheel_rad_s": odo.wheel_rad_s,
                "wheel_distances_m": odo.wheel_distances,
                "body_velocity": odo.velocity,
                "plan": plan,
                "applied_wheels": self.applied,
                "planned_motor_counts": wheel_targets_to_counts(plan["wheels"], self.config),
                "applied_motor_counts": wheel_targets_to_counts(self.applied, self.config),
                "wheel_channels": self.config.wheel_channels,
                "limits": asdict(self.limits),
                "launch_limits": asdict(self.launch_limits),
                "navigation_bounds": self.navigation_bounds,
                "geometry": asdict(self.geometry),
                "sample_age_s": None if odo.time is None else max(0, time.monotonic() - odo.time),
            }


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
                "/style.css": ("style.css", "text/css; charset=utf-8"),
            }
            if path not in assets:
                self.reply(404, {"error": "Not found"})
                return
            name, mime = assets[path]
            self.reply(200, files("capstone_base").joinpath("web", name).read_bytes(), mime)

        def do_POST(self):
            if self.path != "/api/command":
                self.reply(404, {"error": "Not found"})
                return
            origin = self.headers.get("Origin")
            if (
                origin and urlsplit(origin).netloc != self.headers.get("Host")
            ) or not secrets.compare_digest(self.headers.get("X-Control-Token", ""), runtime.token):
                self.reply(403, {"error": "Use the dashboard on this server to send commands"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("Invalid request size")
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected application/json")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("Expected an object")
                runtime.command(
                    payload.get("action"), payload, payload.get("client_id"), time.monotonic()
                )
            except (ValueError, TypeError, UnicodeError) as error:
                self.reply(400, {"error": str(error)})
                return
            self.reply(200, {"ok": True})

    return Handler


def make_server(runtime, host, port):
    return ThreadingHTTPServer((host, port), make_handler(runtime))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Local encoder odometry and target dashboard")
    parser.add_argument("--backend", choices=("simulation", "hiwonder"), default="simulation")
    parser.add_argument("--config", type=Path, default=Path("config/odometry-demo.json"))
    parser.add_argument("--host", default="127.0.0.1", help="Use 0.0.0.0 for trusted LAN access")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument(
        "--vision-config",
        type=Path,
        help="Optional UDP camera input configuration; corrections require browser alignment",
    )
    parser.add_argument(
        "--allow-motion", action="store_true", help="Permit explicit hardware Apply"
    )
    args = parser.parse_args(argv)
    hardware = args.backend == "hiwonder"
    try:
        if args.allow_motion and not hardware:
            raise ValueError("--allow-motion requires --backend hiwonder")
        if not 1 <= args.port <= 65535:
            raise ValueError("Port must be between 1 and 65535")
        geometry, config, limits = load_settings(args.config, hardware)
        vision = (
            VisionReceiver(VisionConfig(**json.loads(args.vision_config.read_text())))
            if args.vision_config
            else None
        )
        driver = HiwonderController(config) if hardware else None
        runtime = DashboardRuntime(
            geometry,
            config,
            limits,
            driver=driver,
            allow_motion=args.allow_motion,
            config_path=args.config.resolve(),
            vision=vision,
        )
        server = make_server(runtime, args.host, args.port)
        if vision is not None:
            try:
                vision.start()
            except OSError:
                server.server_close()
                raise
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    worker = threading.Thread(target=runtime.run, name="odometry-control", daemon=True)
    worker.start()
    print(f"Dashboard: http://{args.host}:{args.port}", flush=True)
    print("Starts in simulation. Hardware reads enabled: " + str(hardware), flush=True)
    print("Hardware Apply allowed: " + str(runtime.allow_motion), flush=True)
    print("Encoder configuration: " + json.dumps(runtime.configuration_report()), flush=True)
    if vision is not None:
        print("Vision input: " + json.dumps(asdict(vision.config)), flush=True)
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
        worker.join(timeout=3)
        server.server_close()
        if vision is not None:
            vision.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if worker.is_alive():
            print("Control thread did not finish; hardware stop could not be confirmed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
