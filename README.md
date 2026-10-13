# Capstone mecanum base

Develop in Ubuntu-24.04 WSL with Python 3.12 and uv. The default CLI calculates
wheel targets with a fake controller. An optional Hiwonder I2C backend uses
smbus2 on the Pi. Hardware has not been tested; the expected board is v1.3.

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

## Environment and files

- pyproject.toml declares dependencies; uv.lock records resolved versions.
- .venv is the isolated project environment; use uv sync --locked to reproduce it.
- The pi extra contains smbus2. It is loaded only when opening a real bus.
- src/capstone_base/kinematics.py: X-layout inverse kinematics and proportional limits.
- src/capstone_base/controller.py: shared interface and fake controller.
- src/capstone_base/hiwonder_i2c.py: hardware adapter and validated motor configuration.
- src/capstone_base/cli.py: fake demo or short, explicit hardware command.
- config/demo.json: unmeasured example geometry for offline tests.
- tests/: movement and mocked I2C protocol/error-path tests.

Inputs are vx forward (m/s), vy left (m/s), wz counterclockwise (rad/s).
Wheel targets use rad/s in FL, FR, RL, RR order. The hardware adapter handles
M1..M4 order, sign changes, and encoder-count command units. ROS integration can
wrap these modules later. Do not copy .venv to the Pi; create a separate Python
3.10+ environment there. ROS and course environments are unchanged.

## Next lab session

1. Record Pi model, OS, architecture, Python version, and SSH details.
2. Enable I2C and user access to /dev/i2c-1; verify board v1.3 and signal wiring.
3. Confirm motor profile, global encoder polarity, and counts per wheel revolution.
4. Map motor channels/signs and encoder signs using short lifted-wheel tests.
5. Measure chassis geometry and verify physical speed and stopping behavior.
6. Add local stale-command timeout handling before continuous teleoperation/ROS.

The project repository is [NateMG4/NEU-capstone-robot](https://github.com/NateMG4/NEU-capstone-robot).
Version the source, configuration templates, tests, docs, pyproject.toml and uv.lock.
Deploy with Git or rsync, excluding .venv, caches, .git and local logs. Keep only
one process responsible for the physical motor controller.
