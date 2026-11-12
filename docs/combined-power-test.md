# Drive and pan/tilt power test

The main `capstone-control` dashboard includes a **Drive + servos** tab. It shows
both existing controls side by side on wide screens, or stacked on smaller ones.
Both pages remain loaded so wheel movement can continue while you operate pan
and tilt. Commands are applied independently; this is not a synchronized start
or an automated load cycle. Leaving this tab requests a stop on both systems.
**Stop all movement** stops wheels and both selected servos, preserving torque.
Each embedded page's Stop/keyboard shortcut still controls its own subsystem.
The existing movement permissions, feedback checks and heartbeat timeouts apply.

## Assign the servo IDs first

Do not operate the bus with two factory-ID servos connected. Both usually use
ID 1, and the protocol cannot reliably distinguish duplicate physical IDs.
Stop the dashboards, power down, and disconnect tilt from the bus. With **only
pan connected**, power it and verify its factory ID:

```bash
.venv/bin/capstone-servo-test --backend buslinker --port /dev/ttyS0 --id 1 status
```

Keep pan as ID 1. Power down, disconnect pan, and connect **only tilt**. Power it
and assign ID 2 (persistent across power cycles):

```bash
.venv/bin/capstone-servo-test --backend buslinker --port /dev/ttyS0 --id 1 --new-id 2 status
```

If either servo already has another ID, use its actual ID instead of `--id 1`.
Power down before reconnecting both. In `config/pan-tilt.local.json`, set
`pan.servo_id` to 1 and `tilt.servo_id` to 2, and set the actual serial port.
The dashboard does not discover or assign IDs. See [servo setup](servo.md) for
ID verification and [pan/tilt calibration](pan-tilt-dashboard.md) for travel limits.

## Launch

Offline preview:

```bash
.venv/bin/capstone-control
```

Open **http://127.0.0.1:8080/#combined**.

For hardware with uncalibrated servos, using your existing measured wheel config:

```bash
.venv/bin/capstone-control --host 0.0.0.0 \
  --wheel-backend hiwonder --wheel-config config/robot.local.json \
  --wheel-allow-motion \
  --servo-backend buslinker --servo-config config/pan-tilt.local.json \
  --servo-axes both --servo-test-mode --servo-allow-motion
```

Open **http://10.42.0.1:8080/#combined** (or the Pi's actual address). Select
**Live robot** in the wheel panel, preview a short target, and explicitly Apply.
While driving, use the servo panel's small relative jogs for each axis; test mode
caps these at 2° per request and needs no neutral/travel calibration. Enable
servo torque explicitly if feedback shows it released. Once both joints have
measured calibration, omit `--servo-test-mode` to use absolute pan/tilt targets.
Nothing starts moving merely by opening the combined view.

The servo telemetry displays voltage and temperature. This dashboard does not
measure total supply current, capture short voltage transients, or certify power
capacity; use external supply measurements during the simultaneous load test.
