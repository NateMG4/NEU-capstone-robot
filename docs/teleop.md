# Keyboard teleoperation

## Run on the Pi

~~~bash
cd ~/capstone-base
.venv/bin/python -m pip install -e '.[pi]'  # Refresh commands after uploading an update
.venv/bin/capstone-teleop                  # Simulation: prints commands, no I2C
.venv/bin/capstone-teleop --backend hiwonder --speed 5
~~~

The last command operates real motors. Begin with lifted wheels and an accessible
motor-power disconnect. Use one controller program at a time. From WSL, an
interactive SSH shell works, or use ssh -t user@capstone-pi.local for a remote
command with terminal allocation. uv is not required on the Pi.

## Keys

| Input | Action |
|---|---|
| W / up arrow | Forward |
| S / down arrow | Backward |
| A / left arrow | Strafe left |
| D / right arrow | Strafe right |
| Q / E | Rotate counterclockwise / clockwise, viewed from above |
| 7 / 9 | Forward-left / forward-right diagonal |
| 1 / 3 | Backward-left / backward-right diagonal |
| Space / X | Immediate zero-speed command |
| C / Ctrl+C | Stop and exit |

Hold or repeatedly press a movement key. A terminal sends keypress/repeat bytes,
not key-release events or reliable simultaneous-key state. Releasing a key stops
motion after input expiry: default 0.3 seconds, plus up to one 20 Hz loop interval
under normal operation. The initial OS key-repeat delay may cause a brief pause
between the first press and repeating movement. Use Space/X for an immediate stop.
Use the diagonal keys for combined translation. Unsupported keys stop; unsupported
escape sequences cannot accidentally become direction letters.

## Mapping and units

config/teleop.json records the user-provided output positions:
M1 front-left, M2 rear-left, M3 front-right, M4 rear-right.
wheel_channels is therefore [1,3,2,4] in FL, FR, RL, RR order.
motor_signs starts at [1,-1,-1,1], following Hiwonder's documented forward pattern
[+,-,-,+] in M1..M4 order. The user requested these initial signs for a lifted-wheel
check; they are not physically verified. Check each wheel's direction and roller
arrangement before placing the robot on the floor.

--speed is the maximum wheel-command magnitude in encoder counts per 10ms, not
PWM, RPM, m/s, or a percentage. Default is 5 and the initial config caps it at 10.
The dimensionless mecanum mixer assumes equal wheel/encoder scales and an X roller
layout; diagonal and combined targets are proportionally normalized. Direction
changes ramp at 20 counts/10ms per second. Stop and stale input bypass that ramp.
This bring-up mode needs no guessed geometry. Calibrated SI-unit movement remains
available separately through capstone-base with a completed robot.local.json.

~~~bash
.venv/bin/capstone-teleop --backend hiwonder --config config/teleop.json --speed 5 --timeout 0.3 --rate 20 --acceleration 20
~~~

## Stopping and limits

Startup is zero. Only movement keys renew the command lease. The Pi checks expiry
locally, rather than relying on a later stop message from the laptop. EOF/SSH input
loss, Ctrl+C, C, SIGTERM, SIGHUP, and normal exit attempt zero-speed cleanup. An I2C
fault stops the session rather than silently resuming. Terminal settings are restored
on exit. This is a best-effort user-space loop, not a hard realtime controller.
A frozen/killed Pi process, power loss, or failed I2C bus can prevent a stop; no
controller-side watchdog has been verified. Keep the physical disconnect available.

## Implementation and verification

- teleop.py: pure directional mixer, wheel mapping, acceleration, input lease, key decoder.
- keyboard_teleop.py: terminal input, timed loop, fake/Hiwonder selection and cleanup.
- tests/test_teleop.py: movement patterns, limits, repeated/stale input, EOF/errors,
  signals, escape sequences, terminal restoration, and actual pseudo-terminal runs.

119 tests passed in WSL. A Pi SSH fake-mode run verified ramp-up, timeout-to-zero, clean quit, and terminal
restoration. Real keyboard movement and direction signs still need operator validation.

References: [Hiwonder signed motor examples](https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/3._Arduino_checked.html),
[Python terminal control](https://docs.python.org/3/library/termios.html).
