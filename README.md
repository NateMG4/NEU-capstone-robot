# Capstone mecanum base

Develop in Ubuntu-24.04 WSL with Python 3.12 and uv. The default CLI calculates
wheel targets with a fake controller. An optional Hiwonder I2C backend uses
smbus2 on the Pi. The expected board is v1.3. Limited wheel bring-up and
user-reported single-servo movement are recorded in the hardware guides;
calibrated navigation and loaded two-servo operation still need lab verification.

## Project status and documentation

Start with the [current-state report](docs/current-state.md) for the repository
review dated 2026-11-12, recent work, commit recommendations, integration status,
and prioritized remaining work. The [documentation index](docs/README.md) links
all operating guides. The [UI integration handoff](docs/ui-integration.md)
describes the existing HTTP interface for the teammate's future UI repository.

## Open and run offline

~~~bash
cd ~/dev/capstone-base
code .
uv run capstone-base --vx 0.1
uv run capstone-base --vy 0.1
uv run capstone-base --wz 0.5
uv run pytest -q
uv run ruff check .
uv run ruff format --check .
~~~

These examples never open I2C. The forward example prints +2 rad/s per wheel
using demo geometry, then clears the fake targets. VS Code selects .venv/bin/python
and recommends Python and Ruff extensions. ROS is not needed for these commands.

## Hardware backend

See [Hiwonder setup, API, protocol sources, and limits](docs/hiwonder.md).
The backend provides initialization, closed-loop speed commands, encoder counts,
supply voltage, wheel/channel conversion, explicit stopping, and connection cleanup.
Use config/hiwonder-v1.3.example.json as a template for config/robot.local.json.
The template's nulls must be replaced with real motor settings and measurements.
It is deliberately unusable for motion as supplied.

When the configuration and wiring are ready, on the Pi:

~~~bash
uv sync --locked --extra pi
uv run --extra pi capstone-base --backend hiwonder --config config/robot.local.json --vx 0.02 --duration 0.25
~~~

This explicitly selected hardware command can move real motors. Start with lifted
wheels and an accessible motor-power disconnect. Duration is required and capped at
two seconds. The backend has no background watchdog. The keyboard teleop loop now enforces a
local stale-input timeout; controller-side watchdog support remains unverified.

## Pi commands (no uv required)

Run from ~/capstone-base on the Pi:

~~~bash
.venv/bin/capstone-base --vx 0.1
.venv/bin/capstone-motor-test --help
~~~

The first command stays offline. For a short raw motor-channel test with lifted wheels:

~~~bash
.venv/bin/capstone-motor-test --motor-type 3 --channel 1 --speed 5 --duration 0.5
~~~

This moves real hardware if the controller/motor setup is working. See docs/hiwonder.md
for units, limits, calibration requirements, and the first test result. Full mecanum
movement still requires measured robot.local.json configuration. The raw-channel test
uses the existing controller backend and requires no wheel calibration.

## Connect directly to the Pi over Wi-Fi

The Pi's recorded bench setup broadcasts **Capstone-Pi** with autoconnect enabled.
Association and reboot behavior still need a lab check. Join it from the
laptop, then run `ssh user@10.42.0.1`. The Wi-Fi password and quick instructions
are in Obsidian: **Capstone → Code Reference → Pi - WiFi**.
See [hotspot controls and troubleshooting](docs/pi-hotspot.md). A router or
Ethernet cable is not required for this connection.

## Realtime keyboard control

See [keyboard controls, mapping, timeout behavior and limits](docs/teleop.md).
The user-provided wheel mapping is M1 FL, M2 RL, M3 FR, M4 RR. Direction signs
start with the documented Hiwonder pattern and need a lifted-wheel check.

On the Pi:

~~~bash
cd ~/capstone-base
.venv/bin/capstone-teleop
.venv/bin/capstone-teleop --backend hiwonder --speed 5
~~~

Default mode prints simulated motor commands. The second command operates real motors.
Use WASD/arrows to translate, Q/E to rotate, Space/X to stop, C or Ctrl+C to quit.
Hold/repeat the movement key; terminal input has no key-release events, so the local
0.3-second input timeout provides release/disconnection stopping while the loop runs.
Wheel commands use encoder counts/10ms and do not require unmeasured chassis geometry.

## Main robot dashboard

Run `uv run capstone-control` and open `http://127.0.0.1:8080` for one page with
tabs for wheels/navigation and pan/tilt. Both start in simulation. Use `--host
0.0.0.0` for Pi LAN access. Switching tabs requests a stop on the view being left;
**Stop all movement** requests a stop on both systems. Hardware backends and
motion permission are configured separately with `--wheel-*` and `--servo-*`
flags. The main server runs both control loops; separate dashboard processes
are not needed. See the [dashboard run and configuration guide](docs/dashboard-runbook.md)
for all launch modes, calibration fields, camera setup, and an Obsidian-ready copy.

For simultaneous power testing, choose **Drive + servos** to keep both controls
visible and active. See [combined power test setup](docs/combined-power-test.md)
for assigning distinct pan/tilt IDs and launching with uncalibrated servo jogs.

## Encoder odometry and browser dashboard

See [the odometry dashboard guide](docs/odometry-dashboard.md) for simulation,
encoder calibration, target previews, hardware motion gating, and Pi hosting.
Run `uv run capstone-dashboard` from the repository root, then open
`http://127.0.0.1:8081`. Use `--host 0.0.0.0` on the Pi to open the page from a
laptop at `http://<pi-ip>:8081`, or use an SSH port tunnel. Defaults use virtual
encoders and never open I2C. The live backend requires measured configuration;
target movement additionally requires `--allow-motion` and explicit browser Apply.
The local start pose is the origin; no camera/world reference is assumed.

Optional camera support accepts your partner's existing UDP JSON output. Launch
with `--vision-config config/vision.local.json`, then use **Use camera & align**
in the dashboard before selecting a floor-frame target. Fresh observations correct
encoder drift; missing camera data falls back to encoders in the last aligned
frame. See [camera integration and the Pi network protocol](docs/camera-integration.md)
for setup, exchanged fields, timing, correction limits, and a separate camera Pi.

## Pan/tilt mechanism dashboard

Run `uv run capstone-pan-tilt` and open `http://127.0.0.1:8082` for an offline
two-servo test bench. It provides calibrated yaw/pitch targets, jogs, neutral,
stop, per-servo torque controls, live telemetry, position history, and diagnostics.
Use `--axes pan` or `--axes tilt` to test just the connected servo; the other axis
is disabled and receives no commands. The default `--axes both` requires both servos.
For initial testing before neutral or travel limits are known, add `--test-mode`:
it provides relative jogs up to 2° and torque controls using absolute servo feedback.
Read-only fault recovery works after torque release without requiring calibration.
The browser can record neutral and both travel ends, save them to the local
configuration, and apply calibration without commanding motion. Reduce Time per
step for faster testing; see the guide for the unloaded pitch profile.
The first servo governs pan; the second governs tilt, sharing the existing driver
and one serial connection. See [the pan/tilt dashboard guide](docs/pan-tilt-dashboard.md)
for Pi launch commands and per-axis calibration. Hardware requires an explicit
backend/configuration; movement additionally requires measured limits and
`--allow-motion`. It operates independently of the wheel dashboard.

## Environment and files

The standalone LX-224/BusLinker test is described in
[Servo setup and first movement](docs/servo.md). Install the optional `servo`
extra, then run `.venv/bin/capstone-servo-test` for simulated status. Hardware
status requires explicit `--backend buslinker --port DEVICE`; movement also
requires configured mechanical limits. Use `sweep --amplitude 2 --duration 1`
for repeated back-and-forth motion until Ctrl+C. The working Pi 3 UART port is
`/dev/ttyS0`. The wheel controls remain independent.
For chained servos, use `--ids 1,2,3` before the command to share one connection.
Assign each new servo separately at startup with `--id 1 --new-id 2 status`;
connect only that servo while changing a duplicate factory ID. Assignment is
persistent and verifies the new address. See the servo guide for full commands
and per-servo configuration through `LX224Bus`.

- pyproject.toml declares dependencies; uv.lock records resolved versions.
- .venv is the isolated project environment; use uv sync --locked to reproduce it.
- The pi extra contains smbus2. It is loaded only when opening a real bus.
- src/capstone_base/kinematics.py: X-layout inverse kinematics and proportional limits.
- src/capstone_base/controller.py: shared interface and fake controller.
- src/capstone_base/hiwonder_i2c.py: hardware adapter and validated motor configuration.
- src/capstone_base/cli.py: fake demo or short, explicit hardware command.
- config/demo.json: unmeasured example geometry for offline tests.
- src/capstone_base/servo.py and hiwonder_serial.py: servo contract and shared LX-224 bus.
- src/capstone_base/odometry.py and navigation.py: encoder pose and bounded target control.
- src/capstone_base/vision.py: optional UDP pose receiver and freshness checks.
- src/capstone_base/control_dashboard.py: combined wheel and pan/tilt server.
- tests/: kinematics, drivers, navigation, recovery, HTTP/UDP and browser-control checks.

Inputs are vx forward (m/s), vy left (m/s), wz counterclockwise (rad/s).
Wheel targets use rad/s in FL, FR, RL, RR order. The hardware adapter handles
M1..M4 order, sign changes, and encoder-count command units. ROS integration can
wrap these modules later. Do not copy .venv to the Pi; create a separate Python
3.10+ environment there. ROS and course environments are unchanged.

## Next lab session

1. Verify the recorded Pi 3/UART/hotspot setup and installed code against the actual Pi.
2. Measure wheel geometry, encoder scales, channel/sign maps and stopping behavior.
3. Calibrate pan/tilt travel and verify distinct IDs and loaded stop/hold behavior.
4. Agree on the vision network, publisher destination, robot IDs and synchronized clocks.
5. Test calibrated camera-assisted navigation, then simultaneous drive/servo power load.
6. Obtain the teammate UI repository and agree on command/status contracts.

Keyboard and dashboard loops already implement software command expiry. A
controller-side watchdog and behavior after a frozen/killed process remain unverified.
Track acceptance evidence in the [current-state checklist](docs/current-state.md#remaining-features-and-changes).

The project repository is [NateMG4/NEU-capstone-robot](https://github.com/NateMG4/NEU-capstone-robot).
Version the source, configuration templates, tests, docs, pyproject.toml and uv.lock.
Deploy with Git or rsync, excluding .venv, caches, .git and local logs. Keep only
one process responsible for the physical motor controller.
