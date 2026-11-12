LX-224 servo and chain bring-up

The standalone `capstone-servo-test` command supports simulated operation, hardware
status, individual moves, repeated back-and-forth sweeps, stop, and explicit torque
control, chained servos on one serial connection, and persistent ID assignment
at startup. It does not open the wheel controller or change keyboard teleoperation.
The user confirmed status, torque control, and visible movement on a Pi 3 Model B
using `/dev/ttyS0`. The new sweep mode has been tested offline; its hardware check
is next. Chaining and ID assignment have also been tested offline; their hardware
verification is pending. You do not need to run the automated tests on the Pi
before using these commands.

Your power arrangement is 12 V to the wheel controller, with a separate branch
through the existing buck converter set to 7.4 V, then BusLinker and the servo.
Confirm voltage/polarity at the servo connector and the converter's current
capacity. The LX-224 is rated for 6–8.4 V with 2.4–3 A stall current.
[Manufacturer specifications](https://www.hiwonder.com/products/lx-224).

This robot connects the BusLinker to the Pi's UART headers using jumper wires.
Use the board's Servo/TTL selection for this connection.
[BusLinker TTL setup](https://docs.hiwonder.com/projects/BusLinker/en/latest/docs/2_ESP32_Development.html).
Start with one servo, with the mechanism supported and clear of obstructions.
The driver uses the LX-224's protocol at 115200 baud over either UART or USB.

1. Update the project files on the Pi and install the optional dependency.

   From the project directory, with uv:

   ```bash
   cd ~/capstone-base
   uv sync --locked --extra pi --extra servo
   .venv/bin/capstone-servo-test
   ```

   Without uv, using the existing pip-enabled virtual environment:

   ```bash
   cd ~/capstone-base
   .venv/bin/python -m pip install -e '.[pi,servo]'
   .venv/bin/capstone-servo-test
   ```

   The last command prints simulated status and never opens hardware.
   `python -m capstone_base.servo_test` is also available in the environment.

2. Select the serial port.

   The working port on this Pi 3 is `/dev/ttyS0`. Set that in the local config.
   If the UART is absent after a fresh OS setup, use `sudo raspi-config` →
   Interface Options → Serial Port: disable the serial login shell, enable the
   serial hardware, then reboot and reconnect over SSH. On the Pi 3, header TX
   is GPIO14 (physical pin 8), RX is GPIO15 (pin 10); connect TX to BusLinker RX,
   RX to BusLinker TX, and share ground.
   [Raspberry Pi UART configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts).

   If switching to USB instead, select Servo/USB on the board and identify the
   device with `.venv/bin/python -m serial.tools.list_ports -v`; use a stable
   `/dev/serial/by-id/...` path if available. USB discovery does not identify a
   device connected only through UART jumpers.
   If opening the port gives permission denied, check the device's group and
   your account's group membership. On a Pi using `dialout`, an administrator
   can add your user to that group; log out and back in afterward.
   Close vendor servo software and other programs using this port.

3. Read hardware status.

   On the current Pi:

   ```bash
   .venv/bin/capstone-servo-test --backend buslinker --port /dev/ttyS0 status
   ```

   This reads position, supply voltage, temperature, mode, and torque state.
   Without `--new-id`, it sends no movement, stop, torque-change, or persistent
   configuration writes.
   ID defaults to 1; use `--id N` before `status` for a previously assigned ID.
   A timeout calls for checking the device, ID, external servo power, wiring,
   and the board's communication selection; the command does not scan IDs or retry movements.

4. Set travel limits for your mechanism.

   ```bash
   cp -n config/lx224.example.json config/servo.local.json
   nano config/servo.local.json
   ```

   Set `port`, `min_angle_deg`, and `max_angle_deg`. The angles use the servo's
   absolute coordinate system, not an automatically homed joint coordinate.
   Choose a narrow, physically clear interval containing the reported position.
   For example, **only if status is around 120° and that travel is clear**, use
   `115.0` and `125.0`. These are example limits, not measured robot calibration.
   Leave the other defaults for the first test. The local file is ignored by Git.

   No movement or torque enabling is allowed while the limits are null.
   Defaults remain a 5° maximum step and a two-second maximum movement time.
   `max_step_deg` can now be configured up to 240°; `max_duration_s` up to 30 seconds.
   Your mechanical angle limits still apply. Use `--max-step 12` before the
   subcommand to override the step limit for one session without editing the file.

5. Enable torque only if status reported it off, then try one small move.

   Enabling torque can engage the joint; support the mechanism first:

   ```bash
   .venv/bin/capstone-servo-test --backend buslinker --config config/servo.local.json torque on
   ```

   If torque is already on, skip that command. For a two-degree positive move:

   ```bash
   .venv/bin/capstone-servo-test --backend buslinker --config config/servo.local.json move --delta 2 --duration 1
   ```

   Choose `--delta -2` if the negative direction is the clear one. Each invocation
   moves relative to newly read position; repeating it moves farther again.
   The driver checks the quantized target against the configured limits and a
   fresh position measurement immediately before sending the command.
   It does not automatically return to the starting position.

   You can also use `move --angle 122 --duration 1` for an absolute target when
   that angle is within your limits and maximum step.
   The CLI waits for the requested movement time plus 0.1 seconds, checks
   position readback against the configured tolerance (default 2°), and requests
   stop on exit while preserving torque. Check visible movement as well:
   a readback within tolerance alone does not prove a small movement occurred.

6. Repeat back and forth until Ctrl+C.

   With torque already on, and at least 4° of clear travel on each side:

   ```bash
   .venv/bin/capstone-servo-test --backend buslinker --config config/servo.local.json --port /dev/ttyS0 --max-step 12 sweep --amplitude 4 --duration 1
   ```

   `--amplitude 4` means approximately 4° either side of the initial position,
   about 8° from end to end. Both endpoints stay fixed for the session; there
   is no cumulative drift. `--duration 1` is the movement time for each leg,
   followed by 0.1 seconds settling and feedback checks.

   The full endpoint-to-endpoint span must fit `max_step_deg`; allow some margin
   for angle quantization and feedback variation. The example uses 12° for this
   limit. Both endpoints must also fit `min_angle_deg` and `max_angle_deg`.
   The CLI validates both endpoints before the first movement.

   Add `--cycles 3` to run three high/low pairs and exit; the default repeats
   until Ctrl+C. On stopping, it holds the last position rather than returning
   to the center. Any communication, health, or position-check failure ends the
   session and attempts a stop.

7. Stop or release explicitly.

   ```bash
   .venv/bin/capstone-servo-test --backend buslinker --config config/servo.local.json stop
   .venv/bin/capstone-servo-test --backend buslinker --config config/servo.local.json torque off
   ```

   Stop preserves torque. Torque off releases the joint, so support any load
   before using it. Both operations are available without travel calibration.
   Ctrl+C, SIGTERM, and SIGHUP attempt cleanup during an active movement.
   A lost serial connection or a killed/frozen process can prevent stopping.

Assign IDs at startup before building a chain:

Every connected servo needs a distinct ID. New servos default to ID 1; connect
**one servo at a time** while assigning IDs so a command cannot rename multiple
servos sharing the same address. The `--new-id` option runs before the selected
command, defaults to a status read afterward, and requires no travel limits:

```bash
# Only the servo being assigned should be connected for this step.
.venv/bin/capstone-servo-test --backend buslinker --port /dev/ttyS0 --id 1 --new-id 2 status
```

Repeat with each servo, choosing a different new ID. IDs are integers from 0 to
253; broadcast addressing is not used. ID changes persist across power cycles
according to the [Hiwonder protocol](https://engineering.purdue.edu/477grp4/Team/journal/img%20-%20Juho/week7/servo%20bus%20protocol.pdf).
The driver reads the current ID, checks whether the new address already responds,
writes the new ID once, and verifies readback at the new address. An unchanged ID
is read and verified without another persistent write. Do not leave `--new-id`
in routine launch commands. Use `--id 2` on subsequent starts, or update
`servo_id` in your local config; the command does not rewrite your JSON file.

An existing responding or locally configured ID is rejected. Duplicate physical
IDs cannot be reliably detected by the protocol, so the response check does not
replace assigning new servos separately. A failed write or readback aborts the
selected command. Inspect both the old and new addresses before continuing;
the command does not retry an uncertain persistent write. ID assignment is
rejected during an active motion session; stop all servos on the bus first.

Operate the chain:

When only one servo can be connected, the existing driver and CLI need no second
servo. Select just the connected servo's actual ID with `--id` (including ID 2
with ID 1 disconnected). For example, read only the tilt servo:

```bash
.venv/bin/capstone-servo-test --backend buslinker --port /dev/ttyS0 --id 2 status
```

The command addresses only that ID. Single-servo movement, sweep, stop, and torque
commands work the same way; use that mechanism's measured limits in `--config`.
Do not select disconnected IDs with `--ids`. For browser testing, launch
`capstone-pan-tilt --axes pan` or `--axes tilt`; see
[single-servo dashboard setup](pan-tilt-dashboard.md#test-one-connected-servo-at-a-time).

Once IDs are unique, connect the servos in the chain to the same BusLinker and
use a single process and serial port. Keep the existing 7.4 V power arrangement
and size its current capacity for all attached servos.

```bash
# Read every selected servo; no travel calibration required.
.venv/bin/capstone-servo-test --backend buslinker --port /dev/ttyS0 --ids 1,2,3 status

# Control just one servo on the connected chain.
.venv/bin/capstone-servo-test --backend buslinker --config config/servo.local.json --id 2 move --delta 2

# Apply the same relative move to each selected servo.
.venv/bin/capstone-servo-test --backend buslinker --config config/servo.local.json --ids 1,2,3 move --delta 2

# Stop all selected servos, preserving torque.
.venv/bin/capstone-servo-test --backend buslinker --port /dev/ttyS0 --ids 1,2,3 stop
```

`--id` and `--ids` are alternatives; put either before the command. `--ids`
also works with `sweep` and `torque on/off`. Group moves use each servo's own
initial position for `--delta`; sweeps keep a separate fixed center per servo.
An absolute `--angle` applies the same target to all selected servos. The CLI
uses the same configured travel and step limits for each selected ID, so use
group motion only when those limits are appropriate for every mechanism.
All targets and initial feedback are checked before the first movement.
Movement commands start sequentially; this is not synchronized motion.
Ctrl+C and faults attempt to stop every active servo in the session. A serial
communication fault invalidates initialization for the whole shared bus.
Read-only status sessions still send no actuator writes.

For different per-servo limits or independent targets, share an `LX224Bus` in
Python. Each controller uses its own `ServoConfig`; the bus owns one connection
and serializes whole request/reply transactions:

```python
from dataclasses import replace

from capstone_base.hiwonder_serial import LX224Bus
from capstone_base.servo import ServoConfig

first_config = ServoConfig(port="/dev/ttyS0", servo_id=1, min_angle_deg=115, max_angle_deg=125)
second_config = replace(first_config, servo_id=2, min_angle_deg=110, max_angle_deg=130)

with LX224Bus(first_config) as bus:
    with bus.controller(first_config) as first, bus.controller(second_config) as second:
        print(first.read_status())
        print(second.read_status())
        # When motion is needed, initialize each controller first. Existing
        # move_to(), stop(), and set_torque_enabled() methods address its ID.
```

The angles above are examples, not measured calibration. Enter the bus before
its controllers. Closing a controller leaves the shared connection available;
closing the bus cleans up all its controllers. For a standalone controller,
`LX224Controller(config)` continues to own and close its own connection.
`read_id()` and `set_new_id(new_id)` are available on both hardware and fake
controllers. A successful ID change requires reinitialization before motion.

Implementation notes:

- `servo.py` contains configuration, the typed interface, and the fake backend.
  The fake reports a 120° initial position and immediately changes its reported
  position on movement; it does not simulate load or motor dynamics.
- `hiwonder_serial.py` contains `LX224Controller`, which opens pySerial lazily.
  Construction/opening sends no servo commands. `initialize()` reads and validates
  status without changing mode, torque, calibration, or position.
- Use a context manager. Movement requires initialization, position mode, enabled
  torque, configured limits, 6–8.4 V feedback, and temperature no higher than the
  configured threshold (default 60°C). These software checks supplement the
  physical setup; voltage feedback cannot protect a servo already overpowered.
- Packets use the vendor LX protocol, including checksums, little-endian fields,
  signed position feedback, stop, and load/unload commands.
  [Hiwonder protocol document](https://engineering.purdue.edu/477grp4/Team/journal/img%20-%20Juho/week7/servo%20bus%20protocol.pdf).
- The reader handles noise, fragmented replies, request echoes, and mismatched
  frames under a fixed response deadline; writes have their own bounded timeout.
  Buffered stale replies are discarded before each request. This protocol has
  no sequence numbers, so a delayed matching reply cannot always be distinguished
  from a new one. Use one controlling process and reinitialize after faults.
- A communication fault latches motion off for every controller on the bus.
  For each controller that has issued movement or enabled torque, the driver
  makes one bounded best-effort stop attempt.
  Read-only status failures never send actuator writes. There are no automatic
  movement retries, reconnection, homing, or automatic persistent writes.
  Persistent ID assignment requires explicit `--new-id` / `set_new_id()`.
- A successful serial write means bytes were sent, not that the servo accepted
  or completed the command. Movement uses position readback; torque control checks
  torque readback. A stop request has no direct acknowledgement.
- Owned serial ports close on exit, including errors. Injected ports remain
  caller-owned. Calls are serialized within a process; this standalone tool
  should not run serial reads inside the wheel teleop loop.

For development, run `uv run pytest -q`, `uv run ruff check .`, and
`uv run ruff format --check .`. Automated tests exercise the protocol, validation,
failure handling, and CLI without connected hardware. Physical verification of
the sweep behavior, chained servos, and ID assignment on this purchased hardware
remain the next lab steps.
