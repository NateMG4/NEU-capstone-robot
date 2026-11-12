"""LX-224 chain bring-up and startup ID assignment; simulation is the default."""

import argparse
import json
import signal
import sys
import time
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path

from .hiwonder_serial import LX224Bus, LX224Controller
from .servo import FakeServoController, ServoConfig, _number, validate_status


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def _print_status(status, servo_id):
    print(
        f"ID {servo_id} | Position: {status.position_deg:.2f} deg | "
        f"Supply: {status.voltage_v:.3f} V | "
        f"Temperature: {status.temperature_c} C | "
        f"Mode: {'position' if status.mode == 0 else 'motor'} | "
        f"Torque: {'on' if status.torque_enabled else 'off'}",
        flush=True,
    )


def _move_and_check(controllers, angles, duration):
    # Preflight the whole chain before moving any servo. Each driver checks fresh
    # feedback again immediately before its own write. Moves start sequentially.
    targets = []
    for controller, angle in zip(controllers, angles, strict=True):
        config = controller.config
        position, milliseconds = config.validate_move(angle, duration)
        before = controller.read_status()
        validate_status(before, config, require_torque=True)
        _number(abs(position * 0.24 - before.position_deg), 0, config.max_step_deg, "step size")
        targets.append((position * 0.24, milliseconds))
    for controller, angle, (target_deg, milliseconds) in zip(
        controllers, angles, targets, strict=True
    ):
        print(
            f"ID {controller.config.servo_id}: requesting {target_deg:.2f} deg "
            f"over {milliseconds / 1000:g}s",
            flush=True,
        )
        controller.move_to(angle, duration)
    time.sleep(max(ms for _, ms in targets) / 1000 + 0.1)
    for controller, (target_deg, _) in zip(controllers, targets, strict=True):
        config = controller.config
        after = controller.read_status()
        validate_status(after, config, require_torque=True)
        error_deg = abs(after.position_deg - target_deg)
        _print_status(after, config.servo_id)
        if error_deg > config.position_tolerance_deg:
            raise RuntimeError(
                f"Target not reached for ID {config.servo_id}: error {error_deg:.2f} deg exceeds "
                f"{config.position_tolerance_deg:g} deg tolerance"
            )
        print(f"Position readback within tolerance: error {error_deg:.2f} deg")


def _sweep_targets(config, center, amplitude, duration):
    """Validate both fixed endpoints before issuing the first move."""
    high, _ = config.validate_move(center + amplitude, duration)
    low, _ = config.validate_move(center - amplitude, duration)
    _number((high - low) * 0.24, 0.24, config.max_step_deg, "full sweep span")
    return high * 0.24, low * 0.24


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="LX-224 bring-up (default: fake status). Put connection options before command."
    )
    parser.add_argument("--backend", choices=("fake", "buslinker"), default="fake")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--port", help="Actual UART or USB serial device; overrides config")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--id", type=int, dest="servo_id", help="Unicast ID; default 1")
    selection.add_argument("--ids", help="Comma-separated chain IDs, e.g. 1,2,3")
    parser.add_argument(
        "--new-id", type=int, help="Persist a new ID at startup, then run command (one servo only)"
    )
    parser.add_argument(
        "--max-step",
        type=float,
        help="Override max_step_deg for this session (up to 240); angle limits still apply",
    )
    commands = parser.add_subparsers(dest="command")
    commands.add_parser("status", help="Read mode, torque, voltage, temperature, and position only")
    move = commands.add_parser("move", help="One bounded move; torque must already be enabled")
    target = move.add_mutually_exclusive_group(required=True)
    target.add_argument("--angle", type=float, help="Absolute servo angle in degrees")
    target.add_argument("--delta", type=float, help="Degrees relative to fresh position feedback")
    move.add_argument("--duration", type=float, default=1.0, help="Movement time in seconds")
    sweep = commands.add_parser("sweep", help="Alternate around the starting position until Ctrl+C")
    sweep.add_argument(
        "--amplitude", type=float, default=2.0, help="Degrees either side of the starting position"
    )
    sweep.add_argument("--duration", type=float, default=1.0, help="Seconds per movement leg")
    sweep.add_argument(
        "--cycles", type=int, default=0, help="Number of high/low pairs; 0 repeats until Ctrl+C"
    )
    commands.add_parser("stop", help="Request stop, preserving torque state")
    torque = commands.add_parser("torque", help="Explicit torque change; support the mechanism")
    torque.add_argument("state", choices=("on", "off"))
    args = parser.parse_args(argv)
    command = args.command or "status"
    hardware = args.backend == "buslinker"
    try:
        if args.config:
            settings = json.loads(args.config.read_text())
            if not isinstance(settings, dict):
                raise ValueError("Servo configuration must be a JSON object")
            config = ServoConfig(**settings)
        elif hardware:
            config = ServoConfig()
        else:
            config = ServoConfig(min_angle_deg=115, max_angle_deg=125)
        if args.port is not None:
            config = replace(config, port=args.port)
        if args.servo_id is not None:
            config = replace(config, servo_id=args.servo_id)
        if args.max_step is not None:
            config = replace(config, max_step_deg=args.max_step)
        ids = [int(value) for value in args.ids.split(",")] if args.ids else [config.servo_id]
        if args.ids == "" or len(ids) != len(set(ids)):
            raise ValueError("--ids must contain distinct comma-separated unicast IDs")
        configs = [replace(config, servo_id=servo_id) for servo_id in ids]
        if args.new_id is not None:
            replace(config, servo_id=args.new_id)  # Validate before opening hardware.
            if len(configs) != 1:
                raise ValueError(
                    "--new-id requires one selected servo; assign duplicate IDs in isolation"
                )
        if hardware and config.port is None:
            raise ValueError("Hardware requires --port or a port in --config")
        if command in ("move", "sweep"):
            config.require_limits()
            _number(args.duration, config.min_duration_s, config.max_duration_s, "duration")
        if command == "move":
            if args.angle is not None:
                config.validate_move(args.angle, args.duration)
            else:
                _number(args.delta, -config.max_step_deg, config.max_step_deg, "delta")
        if command == "sweep":
            _number(args.amplitude, 0.24, 120, "amplitude")
            if args.cycles < 0:
                raise ValueError("cycles must be zero (continuous) or a positive integer")
        if command == "torque" and args.state == "on":
            config.require_limits()
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))

    print(
        f"{'HARDWARE' if hardware else 'SIMULATION (no serial connection)'}: "
        f"LX-224 IDs {','.join(map(str, ids))}, {command}",
        flush=True,
    )
    handlers = {}
    try:
        for name in ("SIGINT", "SIGTERM", "SIGHUP"):
            sig = getattr(signal, name, None)
            if sig is not None:
                handlers[sig] = signal.signal(sig, _interrupt)
        with ExitStack() as stack:
            if hardware and len(configs) > 1:
                bus = stack.enter_context(LX224Bus(config))
                controllers = [stack.enter_context(bus.controller(c)) for c in configs]
            else:
                factory = LX224Controller if hardware else FakeServoController
                controllers = [stack.enter_context(factory(c)) for c in configs]
            if args.new_id is not None:
                controller = controllers[0]
                old_id = controller.config.servo_id
                controller.set_new_id(args.new_id)
                print(
                    f"ID assignment verified: {old_id} -> {controller.config.servo_id}", flush=True
                )
            if command == "status":
                for controller in controllers:
                    _print_status(controller.read_status(), controller.config.servo_id)
            elif command in ("move", "sweep"):
                before = [controller.initialize() for controller in controllers]
                for controller, status in zip(controllers, before, strict=True):
                    _print_status(status, controller.config.servo_id)
                if command == "move":
                    angles = [
                        args.angle if args.angle is not None else status.position_deg + args.delta
                        for status in before
                    ]
                    _move_and_check(controllers, angles, args.duration)
                else:
                    endpoints = [
                        _sweep_targets(c.config, s.position_deg, args.amplitude, args.duration)
                        for c, s in zip(controllers, before, strict=True)
                    ]
                    for controller, (high, low) in zip(controllers, endpoints, strict=True):
                        print(
                            f"ID {controller.config.servo_id}: "
                            f"sweeping {low:.2f} to {high:.2f} deg",
                            flush=True,
                        )
                    print(
                        f"{args.cycles or 'unlimited'} cycles. Ctrl+C stops; torque stays on.",
                        flush=True,
                    )
                    cycles = 0
                    while args.cycles == 0 or cycles < args.cycles:
                        _move_and_check(controllers, [high for high, _ in endpoints], args.duration)
                        _move_and_check(controllers, [low for _, low in endpoints], args.duration)
                        cycles += 1
            elif command == "stop":
                # Try every selected servo even if an earlier stop write fails.
                first_error = None
                for controller in controllers:
                    try:
                        controller.stop()
                    except OSError as error:
                        if first_error is None:
                            first_error = error
                if first_error is not None:
                    raise first_error
                print("Stop command sent; physical stopping is not confirmed")
            elif command == "torque":
                if args.state == "on":
                    for controller in controllers:
                        controller.initialize()
                for controller in controllers:
                    controller.set_torque_enabled(args.state == "on")
                    _print_status(controller.read_status(), controller.config.servo_id)
        if command != "status":
            print("Session closed; torque is preserved unless explicitly switched off")
    except KeyboardInterrupt:
        print("\nInterrupted; applicable stop/close cleanup attempted", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Servo test failed: {error}", file=sys.stderr)
        return 1
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
