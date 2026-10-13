"""Offline demo or explicitly selected, time-bounded Pi hardware command."""

import argparse
import json
import math
import signal
import time
from pathlib import Path

from .controller import FakeController
from .hiwonder_i2c import HiwonderConfig, HiwonderController, wheel_targets_to_counts
from .kinematics import Geometry, wheel_speeds


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Mecanum wheel control (defaults to simulation)")
    parser.add_argument("--backend", choices=("fake", "hiwonder"), default="fake")
    parser.add_argument("--config", type=Path, default=Path("config/demo.json"))
    parser.add_argument("--vx", type=float, default=0.0, help="Forward velocity, m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="Left velocity, m/s")
    parser.add_argument("--wz", type=float, default=0.0, help="Counterclockwise yaw rate, rad/s")
    parser.add_argument("--duration", type=float, help="Required for hardware: seconds, >0 and <=2")
    args = parser.parse_args(argv)
    hardware = args.backend == "hiwonder"
    try:
        config = json.loads(args.config.read_text())
        if not isinstance(config, dict):
            raise ValueError("Configuration must be a JSON object")
        if hardware:
            if config["simulation_only"] is not False:
                raise ValueError("Hardware requires a measured simulation_only=false configuration")
            if (
                args.duration is None
                or not math.isfinite(args.duration)
                or not 0 < args.duration <= 2
            ):
                raise ValueError(
                    "Hardware requires --duration greater than 0 and at most 2 seconds"
                )
            hardware_config = HiwonderConfig(**config["hiwonder"])
        elif config["simulation_only"] is not True:
            raise ValueError("Use simulation_only=true for the fake demo")
        geometry = Geometry(
            config["wheel_radius_m"], config["wheelbase_m"], config["track_width_m"]
        )
        targets = wheel_speeds(args.vx, args.vy, args.wz, geometry, config["max_wheel_rad_s"])
        if hardware:
            # Validate conversion before opening the bus or writing configuration.
            wheel_targets_to_counts(targets, hardware_config)
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        parser.error(str(error))

    if not hardware:
        controller = FakeController()
        try:
            controller.set_wheel_speeds(targets)
            print("SIMULATION ONLY: example geometry; no motors connected")
            _print_targets(controller.targets)
        finally:
            controller.stop()
        print("Stop: all simulated wheel targets returned to zero")
        return 0

    previous_handler = signal.signal(signal.SIGTERM, _interrupt)
    try:
        with HiwonderController(hardware_config) as controller:
            controller.initialize()
            print(f"HARDWARE: commanding wheels for {args.duration:g} seconds", flush=True)
            _print_targets(targets)
            controller.set_wheel_speeds(targets)
            time.sleep(args.duration)
        print("Zero-speed command sent; I2C connection closed")
    except KeyboardInterrupt:
        print("Interrupted; stop/close cleanup attempted")
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Hardware operation failed: {error}")
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
    return 0


def _print_targets(targets):
    for label, value in zip(("FL", "FR", "RL", "RR"), targets, strict=True):
        print(f"{label}: {value:+.3f} rad/s")
