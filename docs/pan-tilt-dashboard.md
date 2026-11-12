# Pan/tilt test dashboard

Run `capstone-pan-tilt` on the computer connected to the BusLinker, then open
the page in a browser. This standalone tool uses the existing LX-224 driver and
does not open the wheel controller. The first joint is pan (yaw); the second is
tilt (pitch). Test one connected servo at a time, or both on one serial connection.
Select connected joints at startup with `--axes pan`, `--axes tilt`, or `--axes both`
(the default).

## Try the dashboard offline

```bash
uv run capstone-pan-tilt
# Or use the module directly with the existing environment:
.venv/bin/python -m capstone_base.pan_tilt_dashboard
# Test a single axis in simulation:
uv run capstone-pan-tilt --axes pan
uv run capstone-pan-tilt --axes tilt
```

Open **http://127.0.0.1:8082**. Simulation is the default and never opens serial.
The demo starts both servos at 120° with example limits of 75–165° and neutral
at 120°. Simulated positions change immediately; load and movement dynamics are
not simulated. These demo limits are not robot calibration.

The dashboard includes:

- Yaw and pitch number inputs and sliders. Editing them only previews a pose;
  **Apply yaw & pitch** sends an explicit movement request.
- Independent positive/negative jog buttons, using fresh measured position;
  **Use current pose** copies feedback into the inputs without moving anything.
- **Move to neutral**, which requests the enabled axes' configured zero angles. This is not
  a homing procedure or physical calibration.
- **Stop** / **Escape**, which requests stop on every enabled ID and preserves torque.
- Separate torque enable/release buttons for each servo. Release can let a joint
  fall under its load; support the mechanism first. Changing either servo's
  torque stops any movement of the chain.
- Live joint and absolute servo angles, voltage, temperature, position/motor
  mode, torque state, and sample age. Joint views and a 60-second feedback chart
  help verify direction and travel. The views show independent joint angles,
  not a measured camera or world orientation.
- Configuration inspection, session events, JSON diagnostic/history export,
  and explicit status refresh / fault clearing.

## Test one connected servo at a time

### Initial bring-up without calibration

Use **`--test-mode`** when neutral and mechanical travel limits are not known.
The test session accepts Stop, read-only fault recovery, explicit torque controls,
and relative jogs of at most **2° per request**, starting from fresh feedback.
The default jog increment is 1°. Absolute pose targets and Move to neutral are
hidden and rejected by the server in this mode.

```bash
cd ~/capstone-base
.venv/bin/python -m capstone_base.pan_tilt_dashboard \
  --backend buslinker --config config/pan-tilt.local.json \
  --axes pan --test-mode --allow-motion --host 0.0.0.0
```

Test mode ignores the file's placeholder neutral and travel limits without
rewriting the configuration. It still needs the correct serial port and selected
servo ID. Readouts and history use absolute servo coordinates; the servo's
0–240° command range is a protocol bound, not a measured collision-free range.
Check clearance before each jog. Mode, voltage, temperature, torque, maximum step,
readback verification, and browser heartbeat checks remain active. There is no
automatic homing, torque enabling, or movement on launch.

After a torque event or failed move, **Read status / clear fault** checks fresh
servo health without requiring travel calibration or an enabled torque state.
It clears the old movement fault and leaves the mechanism stopped. If torque is
released, support the mechanism, explicitly Enable torque in the telemetry row,
then try another small jog. An ongoing communication, mode, voltage, or thermal
problem still needs to be resolved before recovery can succeed. No server restart
is required for an ordinary recoverable movement fault.

Use `--axes tilt` when swapping to tilt. Once calibration is established, launch
without `--test-mode` to use calibrated joint targets and neutral.

### Selecting the connected joint

Use this mode while the cables cannot reach both servos. Connect the servo you
want to test and select its joint explicitly. The dashboard disables the other
joint's inputs and torque controls, shows it as disabled, and sends it no reads,
movement commands, or stop commands. Its travel limits can remain null, and its
duration settings do not constrain the connected joint.

For a read-only session, after copying the configuration as described below:

```bash
.venv/bin/capstone-pan-tilt --backend buslinker --config config/pan-tilt.local.json --axes pan --host 0.0.0.0
```

Set measured limits, neutral, direction, and the actual `servo_id` for pan, then
add `--allow-motion` to enable testing:

```bash
.venv/bin/capstone-pan-tilt --backend buslinker --config config/pan-tilt.local.json --axes pan --allow-motion --host 0.0.0.0
```

Stop the server before swapping cables. Connect tilt and restart the same command
with **`--axes tilt`**, using tilt's own calibration and actual servo ID. Selecting
tilt does not rename the servo: its configured ID defaults to 2, but if that servo
still has factory ID 1, set `tilt.servo_id` to 1. The two configuration entries may
use the same ID for separate single-servo sessions. Assign distinct IDs before
connecting both together; two-axis mode rejects duplicate configured IDs.

Once both cables are available, restart with `--axes both` (or omit `--axes`).
Both selected servos must then respond; a missing servo faults the session rather
than silently disabling it. A selected servo disappearing during a single-axis
test also faults and attempts to stop it. There is no automatic switching or ID
discovery.

### Pitch currently uses servo ID 1

Joint role and bus ID are independent. For the current pitch-only bench, set
`tilt.servo_id` to **1** and launch with `--axes tilt`. The inactive pan entry may
also contain ID 1 during separate single-servo sessions. Do not connect both
servos with the same ID or launch `--axes both` until distinct IDs are assigned.

## Movement speed

**Time per step** controls movement duration: smaller values make each step
faster. A 0.2-second leg is commanded five times faster than a one-second leg of
the same size; feedback and settling add time between legs. The page defaults
to 0.5 seconds where the configuration permits it, or the configured minimum.

The unloaded pitch test profile uses `min_duration_s: 0.1` and
`max_step_deg: 10`. Start calibration at **0.5–1 second**; after moves reliably
reach their targets, use **0.2 seconds** or **0.1 seconds** for faster testing.
Calibration mode permits the larger configured steps for long targets;
test mode continues to cap each jog request at 2°. Larger jog increments change
the requested distance; reducing duration changes speed for that distance.

**Jog increment** means the signed distance requested by each plus/minus button
click, measured from fresh position feedback. A 1° increment requests about 1°
per click; repeating clicks moves farther. It does not set neutral or speed.

## Read the radial calibration graphs

Each dial shades the configured travel arc and marks **MIN**, **NEUTRAL**, and
**MAX**, with numeric values below it. The green arm is live feedback, the dashed
gray arm is the editable preview, and the amber arm is an active or failed
requested target when available. An out-of-range preview or reading is red.

The **Radial angle frame** selector switches between joint degrees (neutral is
zero) and absolute servo degrees (neutral is the stored `zero_deg` reading).
Both frames include text showing the servo angle at neutral and at the current
preview. For example, a configured neutral of 191.76° with feedback 120.72° and
direction +1 displays joint pitch -71.04°. A neutral request then asks for a
71.04° total move toward servo 191.76°, in bounded steps. These are coordinate
examples, not recommended calibration settings.

Test mode uses the absolute frame. It does not shade the protocol's 0–240°
range as calibrated mechanical travel. Once all three calibration points are
recorded, their prospective range is shown as a recorded draft until applied.
The shaded range represents configuration and does not verify physical clearance.

**Record neutral here** reads the current pose and makes it the zero reference
when saved/applied. **Move to neutral** commands movement to the configured zero;
its button shows the distance from current feedback. Use measured poses and clear
travel ends rather than random angles.

If a movement faults, inspect the requested step target, measured servo position,
and position error in the fault message. Diagnostics retain the final requested
pose and failed step even after stopping. Clear the fault only after checking
clearance, then try a small jog with a longer step time before a distant target.

The planner leaves settling margin under `max_step_deg`, so a 10° cap usually
gives legs of about 7.9° with the default 2° tolerance. Increasing the cap reduces
the number of stop-and-check legs. It does not expand the calibrated travel range.

## Record calibration from the browser

The **Calibrate neutral & travel** panel works while in test mode. Start with the
correct selected joint/ID; keep the test-mode jogs for approaching the poses.

1. Jog the pitch mount to the pose you want to call zero, such as level. Click
   **Record neutral**. This stores the live absolute servo reading; no conversion
   from a previously displayed signed angle is needed.
2. Jog toward one clear travel end, stopping short of contact or a hard stop.
   Click **Record end A**. Approach the other clear end and click **Record end B**.
   The server sorts the two recorded servo angles into minimum and maximum.
3. Choose the positive joint direction. For positive pitch to mean tilting up,
   choose increasing or decreasing servo angle according to the observed motion.
4. Click **Save calibration**. Neutral must lie between the two recorded ends.
   The server preserves other configuration fields and the other joint's entry.
5. With the servo inside those travel ends, click **Apply saved calibration**.
   This exits test mode and enables calibrated pitch targets and neutral controls
   without changing position or torque. If torque is released, explicitly Enable
   it before requesting movement.

Capturing, saving, and applying calibration send no movement or torque commands.
Fresh feedback and the existing health checks still apply. In a two-axis session,
save both axes before applying. If you record a point again, save it again before
applying. A restart can load the saved calibration by omitting `--test-mode`.

This establishes neutral, travel, and sign for the tested assembly. Repeat the
clearance and load checks after reinstalling the carried load; unloaded travel
alone does not establish clearance or performance with that load attached.

## Read the real mechanism first

Close `capstone-servo-test` and any other process using the same serial port.
For two-axis sessions, make sure the servos have distinct IDs. This dashboard does not assign
IDs or write persistent servo settings; see [the servo guide](servo.md) for ID
assignment with one servo connected at a time.

On the Pi, update/install the project so the new entry point is available:

```bash
cd ~/capstone-base
uv sync --locked --extra pi --extra servo
# Without uv, use: .venv/bin/python -m pip install -e '.[pi,servo]'
cp -n config/pan-tilt.example.json config/pan-tilt.local.json
.venv/bin/capstone-pan-tilt --backend buslinker --config config/pan-tilt.local.json --axes pan --host 0.0.0.0
```

Join the Pi's Wi-Fi and open **http://10.42.0.1:8082**, or use the Pi's actual
address. Binding to `0.0.0.0` permits trusted LAN access; the default loopback
bind is for a local browser or an SSH tunnel. The page and assets work offline.
This is a local control tool, with a same-origin command token, not a public
authenticated service.

This hardware launch reads status only. It sends no movement, torque-change,
or stop writes on startup or ordinary read-only shutdown. Explicit Stop and
Release remain available. Hardware motion / torque enabling additionally
require `--allow-motion` and calibrated travel limits for the enabled axes.

## Calibrate each axis independently

The browser's target inputs set a desired joint angle; they do not set mechanical
limits. If movement is blocked, the page lists every missing requirement above
the target inputs. You can still edit a preview or copy fresh feedback using
**Use current pose** while limits or motion permission are missing; Apply, jog,
and neutral remain blocked. Limits and neutral are loaded from the server's JSON
configuration at startup, so restart after editing that file.

Stop the server, then edit `config/pan-tilt.local.json`:

- `port`: the shared serial device, currently `/dev/ttyS0` on this Pi.
- `servo_id`: pan defaults to 1 and tilt to 2; match the IDs you assigned.
- `zero_deg`: the absolute servo angle at your chosen neutral pose. The supplied
  120° is a placeholder. Reading status does not automatically set neutral.
- `direction`: `1` or `-1` to match the desired positive yaw/pitch direction.
- `min_angle_deg`, `max_angle_deg`: measured, physically clear absolute servo
  angles, between 0 and 240°. Start with narrow intervals containing the current
  position and neutral. Each axis can have different limits.
- `max_step_deg`: maximum commanded change per leg; defaults to 5°.
- `min_duration_s`, `max_duration_s`: allowed time per leg; defaults to 0.5–2 s.
- `position_tolerance_deg`: readback tolerance, default 2°.
- `max_temperature_c`: maximum temperature allowed for movement, default 60°C.

The mapping is **joint angle = direction × (servo angle − zero_deg)**. For example,
with zero 120°, direction −1, and absolute limits 115–130°, the joint range is
−10° to +5°. This is a mapping example, not a suggested travel range.

Both the requested and quantized positions must fit the absolute mechanical
limits. Angle resolution is 0.24°; a value at a non-aligned boundary may be
rejected after quantization. Limits must be measured for the assembled mechanism,
including cable clearance and combinations of pan and tilt. Independent joint
limits do not model collisions or changing load across combined poses.

## Enable testing

Restart with the calibrated local file and explicit motion permission:

```bash
.venv/bin/capstone-pan-tilt --backend buslinker --config config/pan-tilt.local.json --axes pan --allow-motion --host 0.0.0.0
```

Nothing moves on launch or page load. If a servo reports released torque, support
the mechanism and explicitly Enable it. Start with a small jog on each axis,
confirm direction and visible movement, then try yaw/pitch targets.

Larger targets automatically use successive steps no greater than each driver's
configured maximum step, with 0.15 s settling and feedback verification after
each leg. **Time per step** is the duration of each leg, not the full trajectory.
Each step uses fresh initialization feedback and leaves a margin below the step
cap for settling before the driver's final position check. The cap is unchanged.
The servos receive individual commands sequentially; this is not synchronized
motion. Every movement requires fresh, healthy feedback from the enabled servos,
position mode, enabled torque, configured limits, 6–8.4 V, and acceptable
temperature. All enabled axes are preflighted before any is commanded; each driver
also checks fresh feedback before its own write. Final feedback must be within
the configured tolerance; small jogs still need visible verification.

One browser owns the active move. Other browsers can view it and request Stop or
Release, but cannot replace an active target. The visible controlling page sends
a heartbeat every 0.4 s. Closing/hiding it requests stop; loss of its heartbeat
for 2.5 s causes a stop at the next worker iteration. Stop is processed between
bounded serial operations; it is a software stop, not an electrical emergency
stop. A frozen process or lost bus can prevent delivery. Keep physical power
disconnect accessible during bring-up.

Communication, health, and position verification failures latch a fault and
attempt to stop the enabled servos if the session has actuated them. No movement is
retried automatically. Inspect the cause, then **Read status / clear fault**
performs read-only checks and leaves motion stopped. A failed initial port open
requires fixing the connection and restarting the server. Ctrl+C, SIGTERM, and
SIGHUP stop active commands and close the shared bus, preserving torque.
Feedback continues updating while a movement fault is latched; successful status
reads do not clear that fault or resume motion.

## Development checks

```bash
uv run pytest -q
node --check src/capstone_base/pan_tilt_web/app.js
node --test tests/pan_tilt_controls.test.mjs
```

Automated tests cover simulation, shared-bus protocol integration, motion gating,
calibration, single-servo operation with the other servo absent, step limits,
faults, browser ownership/timeouts, stop/release, and HTTP commands. Physical
mechanism testing remains a lab step.
