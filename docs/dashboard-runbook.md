---
type: project
status: draft
created: 2026-11-05
updated: 2026-11-12
tags:
  - ai
  - tool/codex
  - model/gpt-6
  - capstone
  - dashboards
ai_generated: true
ai_tool: codex
ai_model: gpt-6
aliases:
  - Capstone dashboard runbook
---

# Capstone Dashboards — Run and Configuration

Repository status and remaining integration work are tracked in
[the current-state report](current-state.md). For a separate frontend, see the
[UI integration handoff](ui-integration.md). This runbook covers the dashboards
already included in this repository; the teammate's UI repository is not connected.

## Dashboard addresses

Run commands on the Pi from `~/capstone-base`. On WSL, use
`~/dev/capstone-base`. Keep the server running while using its browser page.

| Dashboard | Command | Local browser | Pi hotspot browser |
| --- | --- | --- | --- |
| Main control center | `capstone-control` | `http://127.0.0.1:8080` | `http://10.42.0.1:8080` |
| Wheels/navigation, standalone | `capstone-dashboard` | `http://127.0.0.1:8081` | `http://10.42.0.1:8081` |
| Pan/tilt, standalone | `capstone-pan-tilt` | `http://127.0.0.1:8082` | `http://10.42.0.1:8082` |

The main dashboard starts both control loops and serves both views on **one port**:
`/wheels/` and `/pan-tilt/`. It does not require the standalone servers. Camera
integration is part of the wheels dashboard, not a third camera control page.
The partner's vision debug viewer is a separate application.

> [!info] Main or standalone
> Use the main dashboard **or** the matching standalone hardware dashboards.
> Do not start a second process against the same I2C board or serial bus.
> Simulation needs no motor or servo hardware. Nothing moves automatically on launch.

## 1. Update and install

Connect to the Pi's **Capstone-Pi** Wi-Fi, then:

```bash
ssh user@10.42.0.1
cd ~/capstone-base
.venv/bin/python -m pip install -e '.[pi,servo]'
```

If using uv instead of pip:

```bash
uv sync --locked --extra pi --extra servo
```

The install refreshes console commands, including the new `capstone-control`.
For a new checkout without a virtual environment, use `python3 -m venv .venv`
before the pip command. On WSL for simulation, `uv sync --locked` is sufficient.
The `pi` extra installs I2C support; `servo` installs serial support.

To upload updated code, follow [[02_Areas/Capstone/Code Reference/Pi - Sync|Pi - Sync]].
For hotspot access, see [Pi hotspot setup](pi-hotspot.md).
Use your Pi's actual IP if it is not `10.42.0.1`.

## 2. Main dashboard: toggle between wheels and pan/tilt

### Simulation

```bash
cd ~/capstone-base
.venv/bin/capstone-control --host 0.0.0.0
```

Open **http://10.42.0.1:8080**. Click **Wheels & navigation** or **Pan & tilt**.
Both backends are simulated. The main page shows their backend, motion and fault
status. **Stop all movement** requests a stop on both systems, preserving servo
torque. Switching views first requests a stop on the view being left; it does not
apply a target in the next view. A failed switch request leaves the current view
loaded. Returning to a view reloads its browser controls while keeping server-side
configuration and wheel pose/target state.

The module equivalent, useful before reinstalling entry points, is:

```bash
.venv/bin/python -m capstone_base.control_dashboard --host 0.0.0.0
```

### Read real wheels and servos, with hardware movement disabled

Prepare the local calibration files in sections 3 and 4 first:

```bash
.venv/bin/capstone-control --host 0.0.0.0 \
  --wheel-backend hiwonder --wheel-config config/robot.local.json \
  --servo-backend buslinker --servo-config config/pan-tilt.local.json \
  --servo-axes tilt
```

Use `--servo-axes pan` or `both` for the connected hardware. Wheels still start
in the **Simulation** source: select **Live robot** inside the wheels view to
inspect the physical encoders. The servo view uses the selected backend directly.
Wheel board startup initializes its motor settings and zero output. Hardware
target motion and servo torque enabling remain disabled without their flags.

### Enable measured hardware movement

```bash
.venv/bin/capstone-control --host 0.0.0.0 \
  --wheel-backend hiwonder --wheel-config config/robot.local.json \
  --wheel-allow-motion \
  --servo-backend buslinker --servo-config config/pan-tilt.local.json \
  --servo-axes tilt --servo-allow-motion
```

These permissions are independent. Omit `--wheel-allow-motion` to keep the wheels
read-only, or omit `--servo-allow-motion` to keep servo movement disabled. Either
backend may remain simulated while the other uses hardware. To bring up an
uncalibrated servo, add `--servo-test-mode` as described in section 4.

### Main dashboard flags

| Flag | Purpose / default |
| --- | --- |
| `--host` / `--port` | `127.0.0.1` / `8080`; use `0.0.0.0` for trusted LAN access |
| `--wheel-backend` | `simulation` or `hiwonder` |
| `--wheel-config` | Defaults to `config/odometry-demo.json`; use measured `config/robot.local.json` for hardware |
| `--wheel-allow-motion` | Permit explicit hardware target Apply |
| `--vision-config` | Optional UDP camera config, shared with the standalone wheels dashboard |
| `--servo-backend` | `simulation` or `buslinker` |
| `--servo-config` | Local pan/tilt JSON; required for hardware |
| `--servo-axes` | `pan`, `tilt`, or `both` (default) |
| `--servo-test-mode` | Relative servo jogs up to 2° while calibration is unknown |
| `--servo-allow-motion` | Permit hardware servo movement and torque enabling |

Stop the main process with **Ctrl+C**. It shuts down both workers and closes
their hardware connections. It does not automatically resume a prior move.

## 3. Wheels/navigation dashboard

### Run standalone

Simulation:

```bash
.venv/bin/capstone-dashboard --host 0.0.0.0
```

Read real encoders:

```bash
.venv/bin/capstone-dashboard --backend hiwonder \
  --config config/robot.local.json --host 0.0.0.0
```

Allow explicit target motion:

```bash
.venv/bin/capstone-dashboard --backend hiwonder \
  --config config/robot.local.json --allow-motion --host 0.0.0.0
```

Open **http://10.42.0.1:8081**. The module equivalent is
`.venv/bin/python -m capstone_base.dashboard` with the same arguments.

### Configure measured wheel hardware

Create the local file only if it does not already exist:

```bash
cp -n config/hiwonder-v1.3.example.json config/robot.local.json
nano config/robot.local.json
```

| JSON field | What to configure |
| --- | --- |
| `simulation_only` | `false` for a measured hardware configuration |
| `wheel_radius_m` | Actual wheel radius in meters |
| `wheelbase_m`, `track_width_m` | Front/rear and left/right distances between wheel centers |
| `max_wheel_rad_s` | Conservative launch wheel speed ceiling |
| `hiwonder.bus_number`, `address` | Actual I2C bus/address; example uses bus 1, decimal 52 (`0x34`) |
| `hiwonder.motor_type`, `encoder_polarity` | Correct motor profile and board polarity |
| `hiwonder.wheel_channels` | Channel mapping in FL, FR, RL, RR order; current teleop mapping is `[1, 3, 2, 4]`, verify it |
| `hiwonder.motor_signs`, `encoder_signs` | Independently verified signs for the four normalized wheels |
| `hiwonder.counts_per_wheel_rev` | Measured counts per full wheel revolution for each wheel |
| `hiwonder.max_speed_counts` | Motor board command cap, counts per 10 ms |
| Optional `navigation` object | Persist translation/yaw limits, wheel acceleration, gains, and arrival tolerances |

The example deliberately contains null calibration fields and will fail preflight
until filled in. The demo's geometry/count scales are simulation examples, even
if its `simulation_only` field has been edited. Changing that flag does not calibrate
the robot. `config/teleop.json` is not a metric odometry configuration.

### Operate

1. Select **Simulation** or **Live robot**.
2. Enter X, Y in meters and heading in degrees, or click the map; choose **Preview target**.
3. Check the frame and wheel commands, then enable **Apply target** explicitly.
4. **STOP MOVEMENT** or Space outside form controls stops motion. Changing a
   target/source or updating navigation settings disarms the move.
5. **Reset local origin** clears this source's target, trail and camera corrections.
6. For a latched encoder fault, download diagnostics, then **Recover encoders ·
   stay stopped**. Recovery discards uncertain motion; establish a new origin or
   explicitly align a fresh camera pose before driving again.

The initial `odom` frame is local: X forward, Y left, positive heading CCW.
The map draws X right and Y up. Targets are within 2 m of current estimated pose.
Navigation tuning in the page lasts for this server session; edit JSON to persist
it. The browser renews a 0.75 s movement lease; moves have a 30 s cap.

## 4. Pan/tilt dashboard

### Run simulation or read hardware

```bash
# Simulation, both joints
.venv/bin/capstone-pan-tilt --host 0.0.0.0

# Read the connected pitch servo
.venv/bin/capstone-pan-tilt --backend buslinker \
  --config config/pan-tilt.local.json --axes tilt --host 0.0.0.0
```

Open **http://10.42.0.1:8082**. Use `--axes pan` for pan, or `--axes both` once
both distinct servo IDs are connected. The module equivalent is
`.venv/bin/python -m capstone_base.pan_tilt_dashboard` with the same arguments.

### Configure port, IDs, neutral and travel

```bash
cp -n config/pan-tilt.example.json config/pan-tilt.local.json
nano config/pan-tilt.local.json
```

| JSON field | What to configure |
| --- | --- |
| Top-level `port` | Shared serial device, currently `/dev/ttyS0` on the Pi 3 |
| `pan.servo_id`, `tilt.servo_id` | Actual bus IDs; examples are 1 and 2 |
| Each axis `zero_deg` | Measured absolute servo angle at neutral; 120° in the example is a placeholder |
| Each axis `direction` | `1` or `-1`; joint angle = direction × (absolute angle − neutral) |
| Each axis `min_angle_deg`, `max_angle_deg` | Measured clear absolute travel, inside the servo's 0–240° protocol range |
| Each axis `max_step_deg` | Maximum move per verified leg; example 5° |
| Each axis `min_duration_s`, `max_duration_s` | Allowed time for each move leg; example 0.5–2 s |
| Each axis `position_tolerance_deg` | Readback tolerance, example 2° |
| Each axis `max_temperature_c` | Movement temperature limit, example 60°C |

The current pitch bench uses **ID 1**: set `tilt.servo_id` to `1` when that servo
still has its factory ID and use `--axes tilt`. Role and ID are independent.
Separate single-servo sessions may reuse ID 1; both-axis sessions require distinct
IDs. Disabled joints receive no commands. Stop the server before swapping cables.

### Unknown calibration: test mode

```bash
.venv/bin/capstone-pan-tilt --backend buslinker \
  --config config/pan-tilt.local.json --axes tilt \
  --test-mode --allow-motion --host 0.0.0.0
```

Test mode ignores placeholder neutral/travel without rewriting the file. It
permits relative jogs up to 2° per request and rejects absolute targets/neutral.
Use fresh feedback and small jogs to establish the actual clear range. Support
the mechanism before releasing torque; release can let a loaded joint fall.

In **Calibrate neutral & travel**:

1. Jog to your chosen neutral and **Record neutral**.
2. Approach each clear travel end and **Record end A** / **Record end B**.
3. Choose the positive joint direction.
4. **Save calibration**, then **Apply saved calibration**. These operations record
   or load settings without moving or changing torque. Save both selected axes
   before applying in a two-axis session.
5. The session exits test mode. A later restart loads saved calibration by omitting
   `--test-mode`. Other JSON edits require a restart.

### Calibrated hardware movement

```bash
.venv/bin/capstone-pan-tilt --backend buslinker \
  --config config/pan-tilt.local.json --axes tilt \
  --allow-motion --host 0.0.0.0
```

If feedback reports torque released, explicitly enable it before movement.
Editing inputs previews a pose; Apply, jog and neutral send movement requests.
**Time per step** sets each leg's duration; smaller is faster within configured
bounds. Larger targets use verified bounded legs. **Stop** / Escape preserves
torque. **Read status / clear fault** recovers a resolved movement fault while
stopped. The controlling visible page renews a 2.5 s heartbeat.

## 5. Optional camera integration in the wheels view

```bash
cp -n config/vision.example.json config/vision.local.json
nano config/vision.local.json
```

| Field | Configuration |
| --- | --- |
| `listen_host` | `0.0.0.0` for a separate publisher; `127.0.0.1` for a local test |
| `port` | UDP `5005` |
| `source_ip` | Actual vision publisher IP; example `192.168.50.10` is the partner's laptop, not this robot hotspot |
| `robot_id` | The robot's configured marker identity, usually 1 or 2 |
| `frame_id` | Stable name for the calibrated floor frame, e.g. `floor` |
| `max_age_s` | Maximum total observation age, default 0.5 s |
| `max_position_jump_m`, `max_heading_jump_deg` | Post-alignment disagreement limits, defaults 0.3 m and 25° |

If using `Capstone-Pi` Wi-Fi, use actual `10.42.0.x` publisher/robot addresses;
the example `192.168.50.x` network is a different topology. Synchronize the capture
device and robot epoch clocks using the same chrony/NTP server.

Add `--vision-config config/vision.local.json` to either `capstone-control` or
`capstone-dashboard`. For example, in simulation:

```bash
.venv/bin/capstone-control --host 0.0.0.0 \
  --vision-config config/vision.local.json
```

In the partner's vision repo, point output to this robot:

```bash
python -m vision.main --host 10.42.0.1 --port 5005
```

If processing directly on the camera Pi, add `--source picam`. If processing on
a laptop, that laptop publishes the UDP poses. Images need not enter the robot's
motor loop. The partner's separate debug viewer uses port 8080 by default: choose
another port if running it on the same machine as this main dashboard.

Upstream now also implements person detection and camera streaming, but this
receiver only consumes robot poses. Its default publisher targets a laptop-local
planner, so the robot destination override above is required. The recorded bench
hotspot and the upstream deployment access point are different networks; see
[verified deployment gaps](camera-integration.md#verified-upstream-state-and-deployment-gaps).

The camera sends ID, floor X/Y, heading, capture time, frame number, and sighting
age. The robot aligns at **capture time minus age**, retaining movement since
that sighting. Velocities and listener data are currently unused; no encoder or
motor data is sent back to vision.

Use **Use camera & align** in the selected wheels source. Alignment stops motion
and clears the previous target; select a new floor-frame target and Apply.
Fresh observations correct drift. Lost/stale input continues on encoders in the
last aligned frame. A large discrepancy stops movement and requires explicit
realignment. Camera input cannot clear a latched encoder fault or authorize motion.

## 6. Common startup issues

| Symptom | Check |
| --- | --- |
| `capstone-control` missing | Reinstall editable package, or run the module command |
| Browser cannot connect | Correct Pi IP, `--host 0.0.0.0`, correct port, process still running |
| Address already in use | Stop the old server or choose `--port`; note the vision debug viewer's default 8080 |
| Wheel config rejected | Fill all calibration nulls; use measured wheel geometry/scales |
| Hardware wheels do not move | Launch motion flag, Live robot source, healthy encoders, explicit Apply |
| Servo movement blocked | Correct selected axis/ID, serial port, motion flag, travel calibration, healthy feedback and enabled torque |
| Only pitch is connected | Set the actual `tilt.servo_id` and select `--axes tilt` / `--servo-axes tilt` |
| Camera unavailable | Publisher destination IP, configured source IP, robot ID, UDP 5005, current timestamps and synchronized clocks |
| Tab switch fails | Restore the main server connection; the current view stays loaded if stop request fails |

`0.0.0.0` is a listening address, not a browser destination. Use the Pi IP in the
browser. An SSH tunnel can keep the server on loopback instead:

```bash
ssh -L 8080:127.0.0.1:8080 user@10.42.0.1
```

Run `capstone-control` without `--host 0.0.0.0` in that session and open
`http://127.0.0.1:8080` on the laptop. The embedded views use the same forwarded
port. These dashboards assume a trusted local network; request tokens are not
user accounts. Software Stop requests still depend on a running worker and working bus.

## Review notes

- Hardware limits, neutral, directions and encoder scales must match the assembled robot.
- The integration is tested with simulation, mocked hardware, and loopback HTTP/UDP.
  Loaded movement, physical calibration and network timing still need lab verification.
- Detailed repository guides: `docs/odometry-dashboard.md`,
  `docs/pan-tilt-dashboard.md`, `docs/camera-integration.md`.
- Related note: [[02_Areas/Capstone/Odometry Dashboard - Setup and Use]].
- Related hardware commands: [[02_Areas/Capstone/Code Reference/Wheels - Run]] and
  [[02_Areas/Capstone/Code Reference/Servo - Run]].

## Simultaneous drive and servo testing

Choose the main dashboard's **Drive + servos** tab to keep wheel and pan/tilt
controls visible and active together. Leaving it stops both systems. For first
setup with two factory-ID servos, assign distinct IDs with one connected at a
time before operating the chain. See [combined power test](combined-power-test.md)
for ID assignment, launch commands and the test procedure.
