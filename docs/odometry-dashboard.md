# Encoder odometry and the motion dashboard

The Pi hosts a small Python HTTP server and the control loop. Open its page from
your laptop's browser; the browser does not talk directly to I2C. No cloud hosting,
Node runtime, frontend build, ROS, or camera is needed. The server uses Python's
standard library and the existing optional `smbus2` backend for hardware.

The starting chassis position is `(x=0, y=0, heading=0)` in a local `odom` frame.
Initially +X is forward and +Y is left. The map draws +X right and +Y up. Heading
is counterclockwise. This frame is relative to startup/reset, not a measured room
frame. Wheel slip and yaw drift are not corrected without an independent sensor.

## Run simulation first

From the repository root in WSL or on the Pi:

```bash
uv sync --locked
uv run capstone-dashboard
```

Without uv, after updating/installing the project on the Pi:

```bash
.venv/bin/python -m pip install -e '.[pi]'
.venv/bin/capstone-dashboard
```

The equivalent module entry point works without refreshing an installed script:

```bash
.venv/bin/python -m capstone_base.dashboard
```

Open `http://127.0.0.1:8081` on the machine running the server. Default binding is
loopback. For a browser on another computer on the same trusted LAN:

```bash
.venv/bin/capstone-dashboard --host 0.0.0.0
```

Open `http://<pi-ip>:8081` or `http://<pi-hostname>.local:8081` on your laptop.
`0.0.0.0` is a listening address, not the address to type into the browser.
The server must remain running; an ordinary SSH session ending can terminate it.

Alternatively keep loopback binding and forward the port over SSH:

```bash
ssh -L 8081:127.0.0.1:8081 <user>@<pi-address>
```

Start the server in that SSH session, then open `http://localhost:8081` on your
laptop. SSH is the command/startup or tunnel connection; the page itself is HTTP.
No router port forwarding or internet publication is needed. This is a local lab
server, not an internet-facing service. LAN clients who can open it can control
it; the per-process request token and same-origin checks prevent cross-site
browser commands but are not user authentication. Prefer the SSH tunnel on an
untrusted network.

## Using the interface

1. Start in **Simulation**, which uses virtual wheel counters and example geometry.
2. Click the map or enter target X/Y and heading, then **Preview target**.
3. Inspect the intended straight path, chassis velocity arrow, per-wheel rim
   direction arrows, planned wheel rad/s, and mapped motor counts/10 ms.
4. Toggle **Apply target** on to run the simulated robot. Encoder changes update
   the estimated pose and trace. Planned speeds and applied speeds are separate.
5. Toggle off, click **STOP MOVEMENT**, or press Space outside form controls to stop.
6. **Reset local origin** while stopped clears this source's estimate, target, and
   trace. It establishes a new counter baseline; it does not zero board counters.

Targets are limited to 2 m from the current estimated pose. Changing the target or
source disarms movement; explicitly apply again. Each move is limited to 30 seconds.
The default goal tolerance is 3 cm / 4 degrees with a 0.3-second settling interval.
These are commissioning defaults, not demonstrated physical accuracy. The line is
an intended clear-floor path, not an obstacle-avoiding or guaranteed tracking path.

Simulation integrates quantized count commands into virtual counters. It uses the
same odometry/navigation path as hardware. It has no wheel inertia, friction,
traction, slip, or independent ground truth, so simulated arrival is a software
check, not a physical validation. Speeds labelled applied are requested wheel
rad/s; integer count commands and measured encoder rates reveal quantization.

## Change speed and navigation in the page

The **Speed & navigation** panel provides paired sliders and numeric inputs for
translation speed (m/s) and turning rate (rad/s). Expand **Advanced navigation**
to set the wheel-speed limit, wheel acceleration, position/heading gains, and
arrival tolerances. The heading tolerance is shown in degrees and converted to
radians at the server interface.

Click **Update navigation** to save your edits and recompute the target preview.
Updating stops the current move; explicitly enable **Apply target** again to
resume. **Restore launch values** reloads the startup settings and also stops
movement. Invalid settings are rejected together without partially updating the
controller. The current motion owner must make changes while a move is active.

These controls apply to both simulation and live sources for this server session.
They do not write the JSON configuration. Restarting reloads the launch values.
For persistent changes, update the `navigation` object in your configuration.

The panel shows the current wheel limit, its launch ceiling, and the approximate
straight-line speed ceiling (`wheel radius * wheel rad/s`). Combined translation
and rotation can be slower. Page controls cannot raise the wheel ceiling selected
at launch or bypass motor count caps or hardware motion permission. Raise the
configured wheel limit and restart only after validating the corresponding
hardware settings. Translation/yaw ranges are derived from the launch geometry
and wheel ceiling (and include the launch values); acceleration is tunable up to
12 rad/s², gains to 3/s, position tolerance to 0.25 m, and heading tolerance to
30°, or their respective launch values if larger.

## Calibrate and observe real encoders

Copy `config/hiwonder-v1.3.example.json` to the ignored
`config/robot.local.json`. Fill in measured geometry and all Hiwonder fields:

- Wheel radius, wheelbase and track measured between wheel centers.
- Compatible motor type and board-wide encoder polarity.
- `wheel_channels` in FL, FR, RL, RR order. Current teleop mapping is `[1, 3, 2, 4]`.
- `motor_signs` for positive commanded wheel rotation.
- Independently checked `encoder_signs` for positive measured wheel rotation.
- `counts_per_wheel_rev` for each wheel output, including gearbox and the board's
  actual counting convention. Do not assume motor-shaft PPR equals this value.
- Conservative `max_speed_counts` and `max_wheel_rad_s`.

The example's nulls are intentionally rejected before bus access. Do not change
`simulation_only` on the demo file and treat its invented dimensions/scales as
hardware calibration. `config/teleop.json` does not contain the scales/geometry
required for metric odometry.

```bash
.venv/bin/capstone-dashboard --backend hiwonder \
  --config config/robot.local.json --host 0.0.0.0
```

This explicitly opens/initializes the real controller and reads its encoders, but
does not permit target motion. Select **Live robot** to inspect its readings and
metric odometry. Startup initialization writes zero and motor settings, following
the existing driver's lifecycle. Do not run keyboard teleop, motor tests, or
another hardware dashboard simultaneously: this process must own the controller.

In a hardware-enabled server, **Simulation** remains a separate virtual robot.
It never sends simulated target motion to the real board. The physical counters
continue to be sampled and their estimate is preserved when switching back to
**Live robot**. Each source has its own origin, trail and frame alignment.

Check one wheel at a time, known forward/strafe moves, and yaw in both directions.
Compare distances against a tape measure until the camera is available. A signed
counter changing is not enough to establish the scale or chassis direction.

## Enable real target motion after calibration

```bash
.venv/bin/capstone-dashboard --backend hiwonder \
  --config config/robot.local.json --allow-motion --host 0.0.0.0
```

The server still starts in Simulation with Apply off. Select **Live robot**, select
a short clear-floor target, inspect the preview, then explicitly enable Apply.
This commands real wheels. The existing motor driver handles channel/sign mapping
and count conversion. The dashboard limits wheel rad/s to the lower of the
configured limit and the per-wheel count caps. Start with suitable bench setup and
physical motor-power access, as described in the existing hardware guide.

Optional `navigation` fields in `robot.local.json` override these defaults:

```json
{
  "max_speed_m_s": 0.12,
  "max_yaw_rad_s": 0.4,
  "wheel_accel_rad_s2": 6.0,
  "position_tolerance_m": 0.03,
  "yaw_tolerance_rad": 0.06981317,
  "position_gain": 0.8,
  "yaw_gain": 1.2
}
```

Place that object under the top-level key `navigation` alongside geometry and
`hiwonder`. Wheel acceleration limiting is proportional across all wheels. Stops
on faults, user requests and lease expiry bypass the ramp. The position controller
holds a specified heading independently of travel direction; it does not turn
the robot to face every target. Smooth timed trajectory generation remains a
later extension; this version uses bounded feedback toward a fixed goal.

## Timing, stopping and failure behavior

- The control thread samples/updates at a requested 20 Hz. Browser state polling
  and movement heartbeats are 5 Hz. Neither is hard real time.
- A move requires a heartbeat from the browser that applied it. A 0.75-second
  lease expires independently of GET/state traffic or other tabs. A backgrounded
  or closed page requests stop and ceases renewing the lease. Reconnecting or
  sending another heartbeat never automatically resumes motion.
- Other tabs may stop a move but cannot renew or replace its active command.
- A counter jump, nonfinite input, backward clock, or sample gap over 0.5 seconds
  stops/latches a fault. Use the recovery workflow below after checking the cause.
  Known board counter resets require a new baseline. Plausible small resets cannot
  be detected from counts alone.
- Encoder plausibility uses a loose maximum of `max(20 rad/s, 4 * wheel limit)`
  plus two counts of quantization allowance. It detects gross jumps rather than
  proving encoder correctness or detecting every slip/reset.
- Lease age is checked again after hardware reads. A blocked I2C call, frozen
  process, bus failure, or power loss can still prevent stopping. There is no
  verified hardware watchdog in this driver; the web stop is not an independent
  physical emergency stop. Failed stop attempts are reported as unconfirmed.
- No automatic motion retry follows a fault. Hardware I/O has one owner in the
  control thread; HTTP handlers only set intent under its lock.

## Diagnose and recover an encoder jump

“Implausible encoder jump” is a software rejection, not confirmation that the board
reset. It means a count change is too large for its sampling interval and configured
wheel-output count scale. The guard is unchanged and is checked in count units:

```text
allowed count delta = maximum encoder wheel rad/s * dt * counts_per_wheel_rev / (2*pi) + 2
```

For example, 1,000 counts/revolution, a 50 ms sample and a 20 rad/s plausibility
limit allow about 161 counts. Incorrect count scales, a counter reset, an
inconsistent/corrupt read or sampling behavior can all produce this rejection.
Changing only an encoder sign does not change the absolute size of the delta.

The **Encoder health** panel shows the active configuration path, count scales,
channel/sign mapping and guard limit. Expand **Calibration and last rejected
sample** for the raw before/after counts, per-wheel deltas, allowed deltas, inferred
speeds and sample interval. These describe the rejected sample; the normal pose
and wheel telemetry remain at their last accepted values during the fault.
The command estimate is the magnitude of the last commanded motor count rate
times the sample interval; it is a comparison aid, not a guaranteed physical
measurement. Because command conversion and odometry both use the configured
wheel scale, a wrong scale alone does not necessarily explain large count-rate
disagreements during closed-loop speed control. Compare the raw change with the
command estimate as well as verifying physical counts per wheel revolution.

**Download diagnostics** exports that evidence and configuration as JSON, without
the browser control token. The server also prints a structured `Dashboard fault:`
record on stdout. Keep this report when reconnecting the robot; an old generic
message alone cannot establish which physical cause occurred.

To recover without restarting the dashboard:

1. Inspect the fault and ensure the mechanism is supported/stopped as appropriate.
2. Select **Recover encoders · stay stopped** in the fault banner.
3. The control worker sends zero, reinitializes the connected motor controller,
   and reads counters until three consecutive 50 ms intervals change by no more
   than two raw counts per channel. It allows up to 20 intervals. Requests/status
   polling remain available during initialization.
4. On success, the counters become a new baseline. Apply stays off, the old target
   is cleared, and the last valid pose is retained with an uncertainty notice.
   Intervening movement is discarded; the trail and interpolation history no
   longer span that gap.
5. Use **Reset local origin** at the chosen new starting position before relying
   on the coordinates, then select/preview a new target and explicitly Apply.
   Apply remains disabled while that source's position is uncertain. An explicit fresh
   camera alignment can also re-establish the reference after recovery.

A failed stop, initialization, read, or stability check leaves the fault latched.
There is no automatic retry or return to motion. Startup connection/initialization
errors also leave the control worker available for an explicit recovery attempt.
The recovery button does not repair wiring, calibrate counts, or waive jump limits.
The original fault report remains downloadable after recovery.

The command previously shown without `--config` loads `config/odometry-demo.json`.
The configuration path and calibration are now printed at startup and displayed
in the page. The original 1,000-count scale and geometry in that file were invented
for simulation; marking a file `simulation_only=false` does not measure them.
Use `--config config/robot.local.json` for a separately measured configuration.

If the jump keeps recurring, capture the new diagnostics before changing limits.
Verify actual counts per full wheel revolution, watch whether counters reset when
power sags or another process initializes the board, and check whether the raw
discontinuity repeats near byte boundaries or with irregular sample intervals.
Those are diagnostic hypotheses, not confirmed defects in the current hardware.

## Odometry and optional camera input

`odometry.py` computes modular raw int32 deltas, maps/signs/scales them, and uses
X-layout mecanum forward kinematics. It integrates combined motion with an SE(2)
constant-twist update rather than assuming chassis heading is always zero.
Counts, normalized deltas, cumulative signed wheel travel, measured wheel speeds,
planned speeds and output counts are visible in the page.

The local estimate remains continuous with bounded timestamped history.
`ReferenceFrame` holds a separate absolute-frame transform. The optional
`vision.py` driver receives Capstone_Vision UDP JSON and maps its last-sighting
epoch timestamp (`t - age`) to local monotonic time. Launch with `--vision-config`
and explicitly choose **Use camera & align** to establish the calibrated floor
frame. Alignment disarms movement and clears the old-frame target. Subsequent
fresh, bounded corrections preserve the target and account for movement since
capture. Encoder history is never rewritten.

Missing/stale camera data retains the last alignment and falls back to encoders.
A large camera/encoder discrepancy stops movement and disables corrections until
explicit realignment. Resetting the local origin disables camera corrections.
Encoder faults remain latched regardless of camera availability.

See [camera integration](camera-integration.md) for configuration, exchanged data,
clock synchronization, separate Pi networking, fallback behavior, and limitations.
This is direct alignment with discrepancy gates; it is not covariance fusion.

## Tests and files

```bash
uv run pytest -q
uv run ruff check .
uv run ruff format --check .
```

`tests/test_odometry.py` checks asymmetric scales/mapping, motion directions,
rollover, SE(2) arcs, invalid samples, jumps and capture-time alignment.
`tests/test_navigation_dashboard.py` checks navigation, quantization, simulated
arrival, hardware gating, ownership, lease expiry, delayed reads and faults.
`tests/test_dashboard_recovery.py` checks rejected-sample evidence, stopped recovery,
driver fault reinitialization, unstable/disconnected recovery and startup failures.
`tests/test_dashboard_http.py` checks the real HTTP interface on loopback.
Those HTTP tests need permission to create local sockets.

Assets live in `src/capstone_base/web/` and ship with the Python package. The
dashboard entry point is `capstone_base.dashboard:main`. Existing wheel/servo
bring-up commands remain available.
