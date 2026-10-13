"""Short raw-channel test for initial wiring and encoder mapping."""

import argparse
import math
import signal
import time

from .hiwonder_i2c import HiwonderConfig, HiwonderController


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run one lifted motor briefly, then stop")
    parser.add_argument("--motor-type", type=int, choices=(1, 2, 3), required=True)
    parser.add_argument("--channel", type=int, choices=(1, 2, 3, 4), required=True)
    parser.add_argument(
        "--speed", type=int, default=2, help="Signed encoder counts per 10ms, -10..10"
    )
    parser.add_argument(
        "--duration", type=float, required=True, help="Seconds: greater than 0, at most 1"
    )
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x34)
    args = parser.parse_args(argv)
    if not -10 <= args.speed <= 10:
        parser.error("Speed must be between -10 and 10 counts/10ms")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 1:
        parser.error("Duration must be greater than zero and at most one second")
    try:
        config = HiwonderConfig(
            motor_type=args.motor_type,
            encoder_polarity=0,
            bus_number=args.bus,
            address=args.address,
            max_speed_counts=10,
        )
    except ValueError as error:
        parser.error(str(error))
    values = [0, 0, 0, 0]
    values[args.channel - 1] = args.speed
    previous = signal.signal(signal.SIGTERM, _interrupt)
    try:
        with HiwonderController(config) as controller:
            controller.initialize()
            before = controller.read_encoder_counts()
            print(f"Supply: {controller.read_battery_voltage():.3f} V", flush=True)
            print(f"Before M1-M4: {before}", flush=True)
            print(f"M{args.channel}: {args.speed} counts/10ms for {args.duration:g}s", flush=True)
            controller.set_motor_speed_counts(tuple(values))
            time.sleep(args.duration)
            controller.stop()
            after = controller.read_encoder_counts()
            delta = tuple(
                ((a - b + 2**31) % 2**32) - 2**31 for a, b in zip(after, before, strict=True)
            )
            print(f"After M1-M4:  {after}", flush=True)
            print(f"Delta M1-M4:  {delta}", flush=True)
        print("Zero-speed command sent; I2C connection closed", flush=True)
    except KeyboardInterrupt:
        print("Interrupted; stop/close cleanup attempted")
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Motor test failed: {error}")
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous)
    return 0
