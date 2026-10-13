"""Pi-local keyboard control over an SSH terminal, with a stale-input stop."""

import argparse
import json
import math
import os
import select
import signal
import sys
import termios
import time
import tty
from contextlib import nullcontext
from pathlib import Path

from .hiwonder_i2c import HiwonderConfig, HiwonderController
from .teleop import ZERO, FakeMotorOutput, KeyDecoder, MotorOutput, TeleopState


class TerminalInput:
    def __init__(self, fd):
        self.fd = fd
        self.saved = None

    def __enter__(self):
        if not os.isatty(self.fd):
            raise ValueError("Use an interactive terminal; remote commands need ssh -t")
        self.saved = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd)
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            raise
        return self

    def poll(self, timeout):
        ready, _, _ = select.select([self.fd], [], [], max(0, timeout))
        return os.read(self.fd, 4096) if ready else None

    def __exit__(self, exc_type, exc, traceback):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
        return False


def run_loop(
    controller: MotorOutput,
    state,
    reader,
    *,
    rate=20,
    clock=time.monotonic,
    report=lambda values: None,
):
    if not math.isfinite(rate) or not 5 <= rate <= 100:
        raise ValueError("Update rate must be between 5 and 100 Hz")
    decoder = KeyDecoder()
    deadline = clock()
    previous = None
    controller.stop()
    try:
        while True:
            data = reader.poll(max(0, deadline - clock()))
            if data == b"":
                return  # EOF / closed SSH input.
            if data is not None:
                for event in decoder.feed(data):
                    if event == "quit":
                        return
                    state.accept(event, clock())
                    if event == "stop":
                        controller.stop()
                        previous = ZERO
                        report(ZERO)
            now = clock()
            if now >= deadline:
                values = state.step(now)
                controller.set_motor_speed_counts(values)
                if values != previous:
                    report(values)
                    previous = values
                deadline = now + 1 / rate
    finally:
        controller.stop()


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Realtime keyboard mecanum control; defaults to fake"
    )
    parser.add_argument("--backend", choices=("fake", "hiwonder"), default="fake")
    parser.add_argument("--config", type=Path, default=Path("config/teleop.json"))
    parser.add_argument("--speed", type=int, default=5, help="Wheel speed magnitude in counts/10ms")
    parser.add_argument(
        "--timeout", type=float, default=0.3, help="Input expiry in seconds (0.1..1)"
    )
    parser.add_argument("--rate", type=float, default=20, help="Motor update rate in Hz (5..100)")
    parser.add_argument(
        "--acceleration", type=float, default=20, help="Counts/10ms increase per second"
    )
    args = parser.parse_args(argv)
    try:
        config = HiwonderConfig(**json.loads(args.config.read_text()))
        state = TeleopState(
            config,
            speed=args.speed,
            timeout=args.timeout,
            acceleration=args.acceleration,
            now=time.monotonic(),
        )
        if not math.isfinite(args.rate) or not 5 <= args.rate <= 100:
            raise ValueError("Update rate must be between 5 and 100 Hz")
        if not sys.stdin.isatty():
            raise ValueError("An interactive terminal is required; connect with ssh -t")
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    hardware = args.backend == "hiwonder"
    mode = "HARDWARE" if hardware else "SIMULATION (no I2C connection)"
    print(
        f"{mode}: speed {args.speed} counts/10ms; timeout {args.timeout:g}s; rate {args.rate:g}Hz"
    )
    print("W/S forward/back | A/D strafe | Q/E turn CCW/CW | 7/9/1/3 diagonals")
    print("Arrow keys also drive. Space/X stop. C or Ctrl+C quits. Hold/repeat movement keys.")
    print(
        "Terminal input has no key-release events: releasing stops after the timeout.", flush=True
    )
    if hardware:
        print(f"FL/FR/RL/RR channels: {config.wheel_channels}; signs: {config.motor_signs}")
    driver = HiwonderController(config) if hardware else FakeMotorOutput()
    context = driver if hardware else nullcontext(driver)
    handlers = {}
    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    try:
        for sig in signals:
            handlers[sig] = signal.signal(sig, _interrupt)
        with TerminalInput(sys.stdin.fileno()) as reader:
            with context as controller:
                if hardware:
                    controller.initialize()
                state.last_tick = time.monotonic()
                run_loop(
                    controller,
                    state,
                    reader,
                    rate=args.rate,
                    report=lambda values: print(
                        f"\rM1..M4 counts/10ms: {values}        ", end="", flush=True
                    ),
                )
    except KeyboardInterrupt:
        print("\nInterrupted; stop/close cleanup attempted")
        return 130
    except (OSError, ValueError, RuntimeError) as error:
        print(f"\nTeleoperation failed: {error}", file=sys.stderr)
        return 1
    finally:
        for sig, previous_handler in handlers.items():
            signal.signal(sig, previous_handler)
    print("\nStopped; terminal restored")
    return 0
