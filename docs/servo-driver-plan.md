# Historical implementation plan: BusLinker V3.0 + LX-224

**Current status, reviewed 2026-11-12:** this is the original design record,
not the current backlog. The shared bus, persistent ID assignment, standalone
CLI, pan/tilt dashboard, calibration capture and combined dashboard are now
implemented. Single-servo UART motion was reported by the user; loaded two-axis
and combined hardware acceptance remain pending. Use [servo setup](servo.md),
[pan/tilt operation](pan-tilt-dashboard.md) and the
[current-state checklist](current-state.md#remaining-features-and-changes).
References below to proposed commands, single-servo scope, USB wiring and future
combined controls describe the original plan and are superseded by those guides.

Prepared 2026-10-20. The standalone driver, fake backend, configuration, CLI, and automated tests are now implemented; see [the run guide](servo.md). Physical hardware verification and combined teleoperation remain pending. The plan below records the intended design. Initial scope assumes one positional servo attached to the existing Raspberry Pi; the mechanism and its permitted travel still need to be measured.

Implementation update: the user confirmed status, torque control, and visible motion on a Pi 3 Model B over header UART (`/dev/ttyS0`), with BusLinker in Servo/TTL mode. USB below was the original proposed connection. A repeated `sweep` command is now implemented, with configurable step limits up to 240° and movement times up to 30 seconds; mechanical angle limits remain enforced. Sweep hardware verification and combined teleoperation are still pending.

Chaining update (2026-10-27): `LX224Bus` now shares one serial connection across independently addressed controllers, and the CLI supports `--ids 1,2,3`. Explicit `--new-id N` assigns and verifies a persistent ID at startup before the selected command. Assign duplicate factory IDs with one servo connected at a time. The implementation includes address-conflict checks, per-controller limits, group preflight, and bus-wide communication fault cleanup. These additions supersede the original single-servo/ID-change exclusions below. Chaining and ID-assignment hardware verification remain pending; see [the run guide](servo.md).

The purchases are a [LewanSoul BusLinker V3.0 USB/TTL board](https://www.amazon.com/dp/B0GTW7GR2L) and an [LX-224 serial bus servo](https://www.amazon.com/dp/B0817X3Z3W). Use a Python servo driver over USB serial alongside the existing I2C wheel driver.

```text
Robot application
  ├── HiwonderController → I2C → existing wheel board → four motors
  └── LX224Controller    → USB → BusLinker V3.0 → LX-224 servo
```

The repository already provides useful patterns:

| Existing code | How to use that pattern |
|---|---|
| `src/capstone_base/controller.py` | A small typed interface and a fake implementation; create a separate servo interface because position commands differ from wheel velocities. |
| `src/capstone_base/hiwonder_i2c.py` | Validated configuration, inert construction, explicit opening, injected transport, fault handling, and connection ownership. |
| `src/capstone_base/motor_test.py` | A standalone bring-up command with bounded operation and interrupt cleanup. |
| `src/capstone_base/keyboard_teleop.py` | Later integration point; preserve its wheel stop timing while adding servo controls. |
| `tests/test_hiwonder.py` | Recording transport tests that check actual command bytes and failure paths. |

1. Confirm the electrical and protocol setup.

   The LX-224 is specified for 6–8.4 V, 115200 baud, and 0–240° travel represented by 0–1000 position units. Its default ID is 1, and specified stall current is 2.4–3 A. These specifications are for LX-224, not LX-224HV. [Manufacturer specifications](https://www.hiwonder.com/products/lx-224).

   Current wiring, corrected by the user: a 12 V wall supply splits into two branches. One feeds the motor controller; the other already has a buck converter reducing the voltage to 7.4 V before the BusLinker. This provides the intended nominal voltage for the LX-224; no additional converter is required by this plan.

   During bring-up, verify output voltage and polarity at the servo connector and confirm the existing converter's continuous/transient current capacity for the servo load. The shared wall supply must support the combined motor and servo load. Record both supply and converter ratings; these have not yet been provided.

   ```text
   12 V wall DC supply
     ├── motor controller
     └── buck regulator set to 7.4 V → BusLinker → LX-224
   ```

   The BusLinker documentation specifies a 5–14 V board input. The existing external buck converter supplies the required voltage reduction in this setup. [BusLinker manual](https://docs.hiwonder.com/projects/BusLinker/en/latest/docs/1_BusLinker_V3.0_Servo_Debugging_Board_User_Manual.html).

   Connect Pi USB to BusLinker Type-C and set the jumpers to Servo/USB. Check connector polarity against the actual board and cable. The vendor documents this USB arrangement, but its Pi examples use `HxServoHandler` at 1,000,000 baud for another servo family. Our implementation must use the LX protocol. [BusLinker Pi instructions](https://docs.hiwonder.com/projects/BusLinker/en/latest/docs/4_Raspberry_Pi_Development.html).

   Record the actual Linux device, USB identification, permissions, and a stable device path if available. Do not hardcode `/dev/ttyUSB0` or assume the example's `/dev/ttyACM0`. Treat a successful read from the purchased servo as the compatibility gate before motion.

2. Add the servo contract and configuration.

   Create `src/capstone_base/servo.py` containing `ServoController`, `FakeServoController`, and a validated `ServoConfig`. Proposed operations:

   ```python
   move_to(angle_deg, duration_s)
   read_position_deg()
   read_voltage_v()
   read_temperature_c()
   stop()
   set_torque_enabled(enabled)
   ```

   Define angles initially in the servo's absolute coordinate system. Add joint zero offsets and reversed direction only when the mechanism needs them. The fake records commands and returns explicitly simulated feedback; it must not claim to model real motion.

   Configuration should include port, servo ID, baud rate, read/write deadlines, measured minimum/maximum angles, maximum step, minimum movement time, and shutdown policy. Validate finite numbers, integer IDs, and ordered limits before opening hardware. Leave mechanical limits unset in `config/lx224.example.json`; allow read-only inspection but reject motion until configured. Ignore `config/servo.local.json` in Git.

3. Implement the serial transport and LX protocol.

   Create `src/capstone_base/hiwonder_serial.py`. Add optional `servo = ["pyserial>=3.5,<4"]` dependencies in `pyproject.toml` and update `uv.lock` during implementation. Import pySerial only when opening hardware, preserving offline use without hardware packages.

   Use `serial.Serial` for port access with explicit 8N1 settings, no flow control, bounded reads/writes, and POSIX exclusive access where supported. Inject a minimal serial interface and monotonic clock for tests. pySerial supplies transport operations; we implement servo messages and response validation. [pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html).

   LX packets are `55 55 ID LENGTH COMMAND PARAMETERS CHECKSUM`; length is parameter count plus three, and checksum is the low byte of the complemented sum from ID through parameters. Initial commands: move 1, stop 12, temperature 26, voltage 27, position 28, mode read 30, torque write/read 31/32. Encode multi-byte parameters little-endian. Position feedback is signed 16-bit; voltage is millivolts. [Hiwonder's protocol document, hosted by Purdue](https://engineering.purdue.edu/477grp4/Team/journal/img%20-%20Juho/week7/servo%20bus%20protocol.pdf).

   Assemble fragmented responses, handle leading noise, bound packet lengths, and validate checksum, ID, command, and payload size. Use one deadline for the entire transaction so incoming garbage cannot extend a read indefinitely. Serialize requests; only one transaction may be outstanding. Characterize adapter echo during bring-up and cover observed behavior with fixtures. Do not interpret a successful write as a servo acknowledgement or completed motion.

   Initially support configured unicast IDs only. Keep ID changes, broadcast discovery, persistent calibration writes, gear-motor mode, and coordinated multi-servo moves out of the first implementation.

4. Implement lifecycle, movement limits, and faults.

   Construction and `open()` must send no motion commands. `initialize()` performs read-only checks of mode, position, torque state, voltage, and temperature. Reject unexpected motor mode; do not automatically change configuration, enable torque, or home the servo.

   Convert validated angles with `round(angle_deg * 1000 / 240)`. Enforce both hardware travel and measured mechanical limits, plus a bounded change from fresh feedback and a positive movement duration. Reject invalid targets rather than silently clamping them.

   On communication failure, latch a fault, reject further motion, and make one bounded best-effort stop attempt. Preserve the original exception, report uncertain stopping, and require explicit reinitialization. Do not automatically resend a movement after timeout or reconnect.

   Separate stopping from torque release. Default motion-session cleanup should attempt stop while preserving torque state; releasing a load-bearing joint could let it fall. Provide explicit release for a supported bench setup. A read-only session must close without writing stop or torque commands. Always release owned serial resources, including after errors; leave injected transports caller-owned.

   USB disconnection or process failure can prevent cleanup. Document this limitation and verify actual stop/hold behavior on the bench. Closing the port does not establish that the servo stopped.

5. Add a dedicated bring-up CLI.

   Create `src/capstone_base/servo_test.py` and the `capstone-servo-test` entry point. Default to the fake backend and a read-only status operation. Require explicit `--backend buslinker` for hardware. Provide status, one bounded move, stop, and explicit torque control; avoid an automatic full-range sweep.

   Proposed commands, available only after implementation:

   ```bash
   uv run capstone-servo-test status
   uv sync --locked --extra pi --extra servo
   uv run --extra servo capstone-servo-test --backend buslinker \
     --config config/servo.local.json status
   ```

   A move takes an angle and duration validated against local configuration. Read back position after the expected movement interval, report the error against a configured tolerance, and distinguish “command sent” from “target reached.” Handle Ctrl+C, SIGTERM, and SIGHUP with the applicable cleanup policy.

6. Verify offline, then bring up one servo.

   Add `tests/test_servo.py`, `tests/test_hiwonder_serial.py`, and `tests/test_servo_test.py`. Cover independent known packet examples; position conversion; signed feedback; malformed, fragmented, stale, and wrong-ID responses; transaction deadlines; short/failed writes; resource ownership; fault recovery; limits; and cleanup. Assert that default CLI use never opens hardware and status never sends motion or torque writes.

   Run the existing checks after implementation:

   ```bash
   uv run pytest -q
   uv run ruff check .
   uv run ruff format --check .
   ```

   On the Pi, start with one mechanically supported servo. Verify the supply and USB device, then obtain repeatable position/voltage/temperature readings. Record mode and torque state. Establish a small permitted travel interval, explicitly enable torque if needed, and command one small move away from the current position. Compare physical motion and readback before testing stop, interrupt handling, and reconnection. Verify that reconnection never resumes a previous target. Keep physical stopping observations separate from software test results.

7. Integrate into robot control after standalone acceptance.

   Keep the wheel interfaces and mecanum kinematics unchanged. Add servo controls to teleoperation only after its intended role and key bindings are known. The current `KeyDecoder` treats unknown keys as wheel stops, so new servo keys need deliberate decoding and tests.

   Serial telemetry must not block the 20 Hz wheel loop or its 0.3-second stale-input stop. Use a dedicated servo I/O worker with bounded requests, expiring motion commands, and timestamped telemetry. Keep one owner of the serial port. Define coordinated stop behavior and test wheel stopping with a missing or stalled servo connection before enabling combined hardware operation.

   Finish with `docs/servo.md` covering wiring, configuration, API, operating limits, and observed hardware results, plus a README link. ROS and mechanism-specific motion planning can wrap the driver later.

The recommended delivery sequence is: contract/configuration and fake → protocol/transport and tests → read-only hardware verification → bounded motion CLI and calibration → combined robot control. The standalone software is ready for bench testing; the physical device path, mechanism limits, mounting direction, supply/converter current ratings, and load-appropriate shutdown behavior are the remaining bench inputs.
