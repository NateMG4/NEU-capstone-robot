"""Optional Capstone_Vision UDP pose input; no motor or camera dependencies."""

import ipaddress
import json
import math
import socket
import threading
import time
from dataclasses import asdict, dataclass

from .odometry import Pose, finite

MAX_PACKET_BYTES = 16384


@dataclass(frozen=True)
class VisionConfig:
    listen_host: str = "127.0.0.1"
    port: int = 5005
    source_ip: str | None = "127.0.0.1"
    robot_id: int = 1
    frame_id: str = "floor"
    max_age_s: float = 0.5
    max_position_jump_m: float = 0.3
    max_heading_jump_deg: float = 25.0

    def __post_init__(self):
        ipaddress.IPv4Address(self.listen_host)
        if self.source_ip is not None:
            ipaddress.IPv4Address(self.source_ip)
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("Vision port must be between 1 and 65535")
        if type(self.robot_id) is not int or self.robot_id < 0:
            raise ValueError("Vision robot_id must be a nonnegative integer")
        if (
            not isinstance(self.frame_id, str)
            or not self.frame_id.strip()
            or self.frame_id == "odom"
        ):
            raise ValueError("Vision needs a calibrated frame_id other than odom")
        for name in ("max_age_s", "max_position_jump_m", "max_heading_jump_deg"):
            if finite(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.max_age_s > 0.5 or self.max_heading_jump_deg > 180:
            raise ValueError(
                "Vision max age cannot exceed 0.5 s; heading gate cannot exceed 180 deg"
            )


@dataclass(frozen=True)
class Observation:
    pose: Pose
    timestamp: float  # Last sighting mapped to the receiving process's monotonic clock.
    epoch: float
    frame_time: float
    frame: int
    track_age_s: float


class VisionReceiver:
    """Bounded latest-observation mailbox, with packet and capture-time validation.

    Both machines must synchronize their epoch clocks (e.g. chrony). Remote
    monotonic timestamps cannot be compared across machines. No packet controls
    motors, changes modes, selects goals, or grants a browser movement lease.
    """

    def __init__(self, config: VisionConfig):
        self.config = config
        self.lock = threading.Lock()
        self.latest = None
        self.last_frame_time = None
        self.received = self.accepted = self.rejected = 0
        self.last_error = None
        self.transport_error = None
        self.socket = self.thread = None
        self.quit = threading.Event()

    def ingest(self, data, address, *, now, wall_now):
        """Accept the partner's unchanged JSON schema. Also usable by offline tests."""
        with self.lock:
            self.received += 1
            try:
                if self.config.source_ip is not None and address[0] != self.config.source_ip:
                    raise ValueError("Unexpected vision source IP")
                if len(data) > MAX_PACKET_BYTES:
                    raise ValueError("Vision datagram too large")
                msg = json.loads(data)
                if not isinstance(msg, dict):
                    raise ValueError("Expected a vision object")
                t = finite(msg.get("t"), "frame epoch timestamp")
                frame = msg.get("frame")
                if type(frame) is not int or frame < 0:
                    raise ValueError("Expected a nonnegative frame number")
                if self.last_frame_time is not None and t <= self.last_frame_time:
                    raise ValueError("Duplicate or out-of-order vision frame")
                robots = msg.get("robots")
                if not isinstance(robots, list):
                    raise ValueError("Expected a robots array")
                selected = [
                    r
                    for r in robots
                    if isinstance(r, dict)
                    and type(r.get("id")) is int
                    and r["id"] == self.config.robot_id
                ]
                if len(selected) != 1:
                    raise ValueError("Configured robot missing or duplicated")
                robot = selected[0]
                age = finite(robot.get("age"), "track age")
                if age < 0:
                    raise ValueError("Track age cannot be negative")
                # Tracker.state() holds the last accepted pose; it does NOT
                # extrapolate it to frame time, even though velocity is included.
                epoch = t - age
                delay = wall_now - epoch
                if wall_now - t < -0.005 or not -0.005 <= delay <= self.config.max_age_s:
                    raise ValueError("Stale/future vision timestamp; check clock synchronization")
                if age > self.config.max_age_s:
                    raise ValueError("Robot track is stale")
                pose = Pose(
                    finite(robot.get("x"), "robot x"),
                    finite(robot.get("y"), "robot y"),
                    math.radians(finite(robot.get("heading"), "robot heading")),
                )
                observation = Observation(pose, now - max(0, delay), epoch, t, frame, age)
                self.last_frame_time = t
                # Repeated stale tracks must not be treated as new measurements.
                if self.latest is None or epoch > self.latest.epoch + 0.001:
                    self.latest = observation
                self.accepted += 1
                self.last_error = None
                return True
            except (ValueError, TypeError, UnicodeError, RecursionError) as error:
                self.rejected += 1
                self.last_error = str(error)
                return False

    def observation(self, now):
        with self.lock:
            if (
                self.latest is not None
                and 0 <= now - self.latest.timestamp <= self.config.max_age_s
            ):
                return self.latest
            return None

    def snapshot(self, now):
        with self.lock:
            obs = self.latest
            age = None if obs is None else max(0, now - obs.timestamp)
            return {
                "configured": True,
                "connected": obs is not None and age <= self.config.max_age_s,
                "configuration": asdict(self.config),
                "age_s": age,
                "frame": None if obs is None else obs.frame,
                "track_age_s": None if obs is None else obs.track_age_s,
                "pose": None if obs is None else asdict(obs.pose),
                "received": self.received,
                "accepted": self.accepted,
                "rejected": self.rejected,
                "last_error": self.transport_error or self.last_error,
            }

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self.config.listen_host, self.config.port))
            sock.settimeout(0.1)
        except OSError:
            sock.close()
            raise
        self.socket = sock
        self.thread = threading.Thread(target=self.run, name="vision-udp", daemon=True)
        self.thread.start()

    def run(self):
        while not self.quit.is_set():
            try:
                data, address = self.socket.recvfrom(MAX_PACKET_BYTES + 1)
            except socket.timeout:
                continue
            except OSError as error:
                if not self.quit.is_set():
                    with self.lock:
                        self.transport_error = str(error)
                return
            self.ingest(data, address, now=time.monotonic(), wall_now=time.time())

    def close(self):
        self.quit.set()
        if self.socket is not None:
            self.socket.close()
        if self.thread is not None:
            self.thread.join(timeout=1)
