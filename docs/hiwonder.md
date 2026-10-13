# Hiwonder I2C backend

Expected hardware: Hiwonder 4-Channel Encoder Motor Module, **v1.3 per the user's latest information**. Earlier project notes said v1.4. Check the physical marking in the lab. The implementation follows Hiwonder's published four-channel protocol; it has not been tested against the physical board and does not auto-detect firmware/revision.

## What smbus2 already handles

The backend calls smbus2.SMBus directly for every bus operation:

- write_byte_data: motor type and encoder polarity settings.
- write_i2c_block_data: the four encoded motor-speed bytes, including zero-speed stopping.
- read_i2c_block_data: encoder and voltage bytes.
- close: releases the Linux I2C device handle.

smbus2 and the Linux kernel handle bus addressing, transactions, and acknowledgments.
The driver supplies the Hiwonder register addresses and interprets payloads. The
I2CBus Protocol is only a Python typing interface for substituting a recording bus
in tests; it does not implement a second I2C transport. Python's struct library
decodes signed multi-byte values. smbus2 does not know wheel mappings, encoder
counts per revolution, controller speed units, or when a robot should stop.

SMBus also offers word reads, which could read a two-byte voltage value on a
compatible device. The existing block read follows the vendor example and keeps
its byte count explicit. There is no need for a custom low-level transaction.

Reference: https://smbus2.readthedocs.io/en/latest/

## Install and configure

In the project folder on the Pi (after enabling I2C and obtaining access to /dev/i2c-1):

~~~bash
uv sync --locked --extra pi
cp config/hiwonder-v1.3.example.json config/robot.local.json
~~~

If the Pi does not have uv, an activated, pip-enabled Python 3.10+ venv can install the package with python -m pip install -e '.[pi]'. Use the project's deployment guide for the broader Pi setup. The Pi extra is smbus2. The backend imports it only when opening a real bus; fake-controller use and mocked tests do not need it.

Fill every null in robot.local.json before hardware use:

| Setting | Meaning |
|---|---|
| wheel_radius_m, wheelbase_m, track_width_m | Measured geometry, in meters |
| max_wheel_rad_s | Wheel angular speed cap applied by the mecanum mixer |
| motor_type | Vendor encoder motor profile: 1 TT, 2 N20, 3 JGB37-520; verify actual motor compatibility |
| encoder_polarity | Board-wide encoder polarity, 0 or 1, matching motor profile/wiring |
| wheel_channels | M1..M4 channel for each wheel, ordered FL, FR, RL, RR; each channel used exactly once |
| motor_signs | Per-wheel +1/-1 mapping from positive wheel speed to positive board command |
| encoder_signs | Per-wheel +1/-1 mapping from raw encoder counts to positive wheel rotation |
| counts_per_wheel_rev | Four positive encoder scales at the wheel output, including gearbox and counting convention |
| max_speed_counts | Maximum absolute command in counts/10ms; initial configurable cap 10, not a hardware rating |
| address, bus_number | Default 52 (hex 0x34), bus 1 |

The example contains nulls intentionally and is rejected before opening hardware. Do not use the demo geometry or guessed encoder scales for calibrated motion. Board-wide encoder polarity and per-wheel software signs serve different purposes: software sign changes cannot repair unstable encoder feedback inside the board.

## CLI

The existing command remains a fake demo unless --backend hiwonder is explicit.
Once measured hardware configuration is ready, an example short lifted-wheel test is:

~~~bash
uv run --extra pi capstone-base --backend hiwonder --config config/robot.local.json --vx 0.02 --duration 0.25
~~~

This command moves real motors when run on a connected Pi. Hardware mode requires an explicit duration greater than zero and at most two seconds. It stops before/after initialization, sends the requested speeds, waits the requested duration, and sends zero on exit. Ctrl+C and SIGTERM trigger cleanup. Geometry-based limits and controller-count limits both apply; a count-limit violation is rejected before the CLI opens hardware.

Only one program should own this motor controller. Keep the wheels lifted for initial mapping and retain the motor-power disconnect. Zero is a requested speed, not verified physical stopping.

## First-wheel bring-up without wheel calibration

On the Pi, use the installed virtual environment directly; uv is not required:

~~~bash
cd ~/capstone-base
.venv/bin/capstone-base --vx 0.1
.venv/bin/capstone-motor-test --help
~~~

The first command is simulation-only. The motor-test command below operates real hardware and requires the wheels lifted with an operator at the power disconnect:

~~~bash
.venv/bin/capstone-motor-test --motor-type 3 --channel 1 --speed 5 --duration 0.5
~~~

--channel is M1..M4, not a known physical wheel yet. --speed is signed encoder counts/10ms, capped at +/-10 in this bring-up tool. --duration must be >0 and <=1 second. The command initializes the selected motor profile with global encoder polarity 0, commands only the selected channel, sends zero before reading the final counts, and closes with another zero-speed command. Ctrl+C/SIGTERM also attempt cleanup.

For raw tests, HiwonderConfig requires only motor_type and encoder_polarity. Wheel mapping/signs/scales are optional until high-level wheel-rad/s commands or wheel-ordered encoder reads are requested. Those operations reject missing calibration. This allows initial channel mapping without invented geometry or encoder scales.

First physical test on 2026-10-13: motor profile 3, M1, +2 counts/10ms, 0.25s. Supply was 12.646 V. Encoder counts changed from (-124,-80,-1,0) to (-123,-80,-1,0), delta (1,0,0,0). The operator reported sound but no visible wheel movement. Zero-speed cleanup completed. Useful motion, motor/encoder direction, channel-to-wheel mapping, and scale remain unverified. The +5/0.5s command above is a proposed next operator-run test, not a test already performed.

## Python API

~~~python
import json
from capstone_base.hiwonder_i2c import HiwonderConfig, HiwonderController

settings = json.load(open("config/robot.local.json"))
config = HiwonderConfig(**settings["hiwonder"])
with HiwonderController(config) as controller:
    controller.initialize()
    print(controller.read_battery_voltage())  # volts
    print(controller.read_encoder_counts())  # M1, M2, M3, M4
    print(controller.read_wheel_encoder_counts())  # FL, FR, RL, RR with signs
    controller.stop()
~~~

set_wheel_speeds((fl, fr, rl, rr)) takes rad/s and implements the same interface as FakeController. Conversion is rad/s divided by 2*pi, multiplied by counts/revolution and 0.01, followed by direction/channel mapping and integer rounding. Low speeds can round to zero. Out-of-range commands are rejected, not wrapped to arbitrary byte values.

set_motor_speed_counts((m1, m2, m3, m4)) is the lower-level mapping/calibration API. It bypasses geometry and wheel signs, accepts signed integer counts/10ms, and still enforces the configured count limit. It requires successful initialization. No open-loop PWM mode is provided.

read_encoder_counts returns four signed 32-bit counters. read_wheel_encoder_counts reorders and signs them; it does not unwrap overflow, detect resets, calculate velocities, or provide odometry yet. Use modular deltas and reset detection when adding odometry.

## Lifecycle and faults

Construction is inert. open() opens SMBus but writes nothing. initialize() sends zero, writes motor type, waits 0.5s, writes global polarity, waits 0.5s, and sends zero again. Motion calls require initialization. Use the context manager so exceptions close the connection and attempt stopping.

I2C failures, malformed reads, and invalid motion commands latch a fault and attempt one zero-speed write. Further motion requires explicit successful initialize(). Failures are propagated; a failed stop emits a warning. There are no automatic retries of nonzero motion. stop() is available even before initialization or after a fault. close() attempts stop and releases an owned bus even when stopping fails. Injected test buses remain caller-owned.

There is **no controller watchdog configured or background command timeout** inside this backend. The separate keyboard teleop loop provides local input expiry. The CLI provides bounded commands, but a killed/frozen process, failed bus, or Pi power loss can prevent stopping. The keyboard teleop loop now enforces stale-input expiry locally; other continuous callers, including a future ROS node, must do the same and use a verified hardware watchdog if available. The documented 10ms speed unit is not a timeout or required host command interval.

## Protocol evidence

| Operation | Register | Transfer |
|---|---|---|
| Motor profile | 0x14 | One byte |
| Encoder polarity | 0x15 | One byte |
| Four speed targets | 0x33 | Four signed int8 values in M1..M4 order, encoded as unsigned two's-complement bytes |
| Encoder counts | 0x3C | 16 bytes, four little-endian signed int32 values |
| Supply voltage | 0x00 | Two bytes, little-endian unsigned millivolts |

Sources checked 2026-10-13:

- [Official module download page](https://www.hiwonder.com.cn/store/learn/142.html) links the [vendor package](https://drive.google.com/drive/folders/1ZIbMQo2R2YOgqYN3d9nTgIqJnxlGbR8m).
- The vendor package's Raspberry Pi program [TankDemo.py](https://drive.google.com/file/d/1aKB2iPnrKAHHKk6iZ5R08AK_DVT8-sT6/view) was downloaded and read, never executed. It specifies the register constants, byte motor settings, signed speed examples, 16-byte encoder reads, and voltage conversion. Its native struct.unpack('iiii') on a Raspberry Pi is made explicit as '<4i' here for platform-independent decoding. The sample uses smbus; this implementation uses smbus2's compatible methods.
- [Hiwonder chassis Pi tutorial](https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/4_Raspberry%20Pi_formatted.html) independently uses smbus2 for the same speed-write interface.
- [Hiwonder revision notes](https://forum.hiwonder.com.cn/t/topic/438) describe v1.3 as the baseline and explain hardware revision differences. They do not establish the actual firmware on this robot.

## Verification

Tests use a recording bus and cover exact wire bytes, signed little-endian decoding, nonidentity wheel mapping, unit conversion, invalid input, initialization, cleanup, injected/owned bus handling, failed I2C transfers, fault recovery, and hardware CLI validation. These tests verify software behavior against the published protocol, not electrical or mechanical behavior on the robot.
