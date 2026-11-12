# Capstone robot repository: current state

Reviewed **2026-11-12**, with a local commit follow-up on the same date. This is
a dated assessment of `NateMG4/NEU-capstone-robot` at
`/home/natemg/dev/capstone-base`, including the working tree reviewed before its
feature work was committed locally. Source code, tests, configuration templates,
Git history and the Obsidian `02_Areas/Capstone` notes were reviewed. No robot
was connected or moved in this session.

## General summary

This repository implements the mecanum base and LX-224 speaker pan/tilt controls.
It has grown from a wheel-driver/keyboard project into a local robot-control
service with simulation, encoder navigation, browser dashboards and optional
camera pose correction. The software is useful for integration and bench work;
the evidence does not yet establish calibrated, autonomous operation of the
assembled system.

| Area | Implemented state | Evidence or limitation |
| --- | --- | --- |
| Wheels | Mecanum kinematics, fake backend, Hiwonder I2C, bounded CLI/raw motor test, keyboard teleop | Automated tests; limited 2026-10-13 physical test recorded in `hiwonder.md`, without useful visible wheel motion |
| Servo bus | LX-224 protocol at 115200 baud, bounded motion, status, torque, shared serial bus and persistent ID assignment | Mocked protocol/error-path tests; user reported one-servo UART status, torque and visible motion; chain/ID hardware acceptance pending |
| Pan/tilt | Single/both-axis modes, relative test jogs, joint targets, neutral, calibration capture/save/apply, telemetry and faults | Simulation and mocked hardware tests; physical travel, mounting and loaded operation need measurement |
| Odometry/navigation | Modular encoder integration, local pose/history, bounded pose feedback, limits and recovery diagnostics | Tests and ideal quantized wheel simulation; no obstacle avoidance or demonstrated physical navigation accuracy |
| Dashboards | Wheels :8081, servos :8082, combined control :8080; combined drive/servo view | HTTP and JavaScript tests; independent control workers, motion permissions and leases |
| Vision input | Optional UDP robot-pose receiver, capture-time alignment, discrepancy gates and encoder fallback | Protocol and loopback tests; actual upstream publisher smoke test; deployment/calibration still pending |
| Listener aiming | Upstream can produce a tracked listener; this receiver ignores it | No listener-following planner, automatic pan/tilt aiming or coordinated five-axis controller |
| External UI | Bundled bench dashboards exist | Teammate UI URL and state unknown; no connection implemented |

The Python package requires Python 3.10+. The local environment uses Python
3.12. Offline runtime uses the standard library; hardware extras are `smbus2`
(`pi`) and `pyserial` (`servo`). pytest/Ruff are development dependencies. Node
is only used for browser-code checks. ROS, a database and a task-tracking service
are not required by the current implementation.

## Git state and commit assessment

At review start, branch `main` pointed to `0ab7967` (2026-10-13,
**Initial commit: capstone mecanum robot controller**). It matched the locally
recorded `origin/main`; this is not a claim that the remote robot branch was
refreshed. At that point there was only **one recorded commit**. The follow-up
below records the local commits subsequently created for review.

There were **50 pending files: 4 modified tracked files and 46 untracked files**:

| Items found | Count | Assessment |
| --- | ---: | --- |
| Python modules and browser assets under `src/` | 21 | Commit: required servo, dashboard, odometry, navigation and vision implementation |
| Python/JavaScript tests under `tests/` | 13 | Commit with corresponding implementation |
| Operating/design guides under `docs/` | 8 | Commit after this review's accuracy corrections; retain the old servo plan as historical |
| Configuration templates under `config/` | 4 | Commit examples; keep machine-specific calibration separate |
| `.gitignore`, `README.md`, `pyproject.toml`, `uv.lock` | 4 | Commit: ignore local configs, document and expose commands, declare/lock serial dependency |

**Recommendation: retain and commit all reviewed source, assets, tests, templates,
dependency metadata and documentation, including this review's fixes.** These
are project deliverables rather than disposable local files. No pending file
was identified as a private calibration file or generated cache. `.venv`,
caches, logs and `config/{robot,servo,pan-tilt,vision}.local.json` are ignored
and should stay out of commits. The initial assessment left the changes
uncommitted. The user then authorized local commits, with an explicit instruction
not to push; the follow-up below records that work.

### Local commit follow-up

The reviewed workspace is organized into five dependency-ordered local commits:

| Commit | Change | Validation of the staged snapshot |
| --- | --- | --- |
| `93e0383` | LX-224 bus driver, servo CLI, example, serial dependency and tests | 250 Python tests; lint, formatting and lock checks |
| `579a1f0` | Encoder navigation, UDP vision input and wheel dashboard | 364 Python tests; lint, formatting, lock and browser syntax checks |
| `bf9dace` | Pan/tilt dashboard, calibration workflow and tests | 90 pan/tilt Python tests and browser-control tests; lint, formatting and lock checks |
| `42dfa46` | Shared server, tab/combined views and corresponding tests | Full 472-test Python suite, both JavaScript test files, lint, formatting and lock checks |
| This documentation commit | Operating guides, index, current-state report and UI handoff | Relative document links and diff whitespace checks |

Console entry points, local-config ignores and the combined-dashboard regression
case were staged with the features they depend on. Staged snapshots were exported
to temporary directories for testing without changing working-tree source files.
All changes are local; no push was performed. Sharing/pushing remains pending
the user's review.

Do not commit entry points without the modules/assets/configuration they use.
The two standalone dashboards share `dashboard_http.py`; include that helper
before either dashboard. Keep each resulting commit's checks passing.

## Recent work visible in the workspace

These feature groups were uncommitted during the initial review and are now
covered by the local commits above. Dates in notes describe when work was
discussed or recorded, not proven implementation dates.

1. **Servo bring-up and chaining:** added the LX-224 protocol, read-only startup,
   bounded movement/sweep, torque and fault handling, shared bus ownership and
   persistent servo ID changes. October 20–27 notes describe this work and
   report a working Pi 3 `/dev/ttyS0` connection.
2. **Encoder navigation and camera input:** added wheel-count odometry/history,
   local target feedback, browser visualization/tuning, fault diagnostics and
   stopped recovery; then added UDP floor-pose alignment/correction and fallback.
   The October 29 design notes predate parts of this implemented work.
3. **Pan/tilt and unified bench controls:** added one/two-axis dashboard modes,
   uncalibrated jogs, calibration capture/save/apply, telemetry, recovery and
   a combined wheel/servo server/view. November 5 documentation records these
   launch and combined-power-test flows. Independent worker loops do not imply
   synchronized five-axis planning.
4. **This review:** corrected the shipped odometry demo from
   `simulation_only=false` to `true`; hardware selection without measured config
   is now rejected before opening hardware or HTTP sockets in either wheel
   entry point. Added two regression cases, fixed dashboard formatting, updated
   stale hardware/odometry/vision/network claims, and added status/UI documents.

## Computer vision integration

A fresh temporary clone of
[Jeffrey's Capstone_Vision](https://github.com/jeffreyburt/Capstone_Vision)
was inspected at main
[`67817cfa2374f64a9a76986c56df7e034ea40f82`](https://github.com/jeffreyburt/Capstone_Vision/commit/67817cfa2374f64a9a76986c56df7e034ea40f82)
(2026-10-29, networking support). Its previous commit is `00458f0`
(2026-10-27), the version used by the older Obsidian vision review.

The latest inspected code includes robot marker localization, camera transport,
person detection and listener tracking. Person detection depends on its optional
inference stack and calibration; source availability is not proof of real-room
performance. The old note's claim that real messages always have `user: null`
is now historical. See the upstream
[pipeline](https://github.com/jeffreyburt/Capstone_Vision/blob/67817cfa2374f64a9a76986c56df7e034ea40f82/vision/pipeline.py).

The robot accepts the existing `t`, `frame`, `robots[].{id,x,y,heading,age}`
schema. It maps `t-age` to encoder history, requires explicit alignment and
uses fresh observations to correct its floor/odom transform. It ignores listener
and velocity fields and sends no telemetry back to vision. A camera dropout
continues on encoders in the last aligned frame while a valid movement lease
exists; a large discrepancy disarms movement. Camera input never grants motion.

Remaining deployment gaps:

- **Routing:** vision defaults to laptop-local UDP 5005; robot reception needs
  an explicit robot destination or an agreed forwarder. For two robots, select
  fan-out or tested broadcast rather than assuming both receive one unicast.
- **Network:** the bench `Capstone-Pi` hotspot (`10.42.0.0/24`) differs from
  the upstream master AP (`192.168.50.0/24`). The Pi 3 needs 2.4 GHz; its single
  Wi-Fi interface cannot serve the bench AP and join the master AP simultaneously.
- **Timing/calibration:** synchronize capture and robot epoch clocks, confirm
  chassis marker offsets/front, floor coordinates, robot identity and real latency.
- **Higher-level behavior:** listener-to-target planning, pan/tilt aiming,
  robot-camera/servo-angle metadata bridge and multi-robot coordination are absent.
- **Command channel:** upstream reserves ports 5700–5799 for a controls-team
  protocol. This repository implements dashboard HTTP, not that reserved channel.

The detailed contract and commands are in [camera integration](camera-integration.md).
No teammate source or upstream network document was modified.

## Alignment with the project proposal

The Obsidian proposal describes a broader system than the current robot code:
ROS 2 communication/transforms, centrally computed listening-triangle targets,
Idle/Repositioning/Tracking modes, obstacle-avoidance priority and audio feedback
when a joint cannot reach the requested angle. Those passages describe intended
behavior; they are not implemented simply because the bench controls now work.

The current repository has no ROS node, triangle solver, executive mode machine,
obstacle detector/avoidance layer or audio-system event publisher. The inspected
vision pipeline publishes observations rather than computed speaker target poses.
The team needs to assign the planner and executive responsibilities explicitly.
Current HTTP telemetry exposes joint positions, configured limits and faults,
but no agreed audio-facing contract reports speaker acoustic-center pose or
joint-limit/unreachable-target events.

The later proposal text specifies a **fixed mast**, with a powered vertical lift
deferred. No lift controller is expected in this software baseline. Mounting
geometry and load/stability limits still need to constrain the robot's measured
acceleration and joint travel. Reconcile the proposal's ROS requirement with the
current standard-library HTTP/UDP implementation before selecting extra middleware.

## Remaining features and changes

Use these checkboxes as the lightweight backlog. An item closes only when its
acceptance evidence is recorded; a working simulation does not close a hardware item.

### First: establish a repeatable physical baseline

- [x] **Version the reviewed workspace locally:** create coherent local commits
  preserving required browser assets, example configs and tests.
- [ ] **Review and share the commits:** push only after the user's review, deploy
  through the normal project workflow and record the deployed commit on the Pi.
- [ ] **Wheel calibration:** measure geometry and counts/revolution, verify all
  channel/motor/encoder signs, motor profile and polarity; retain private local
  config and record known-distance/turn accuracy and stopping results.
- [ ] **Servo acceptance:** verify unique IDs on the physical chain, neutral,
  direction and travel on each mounted axis; record loaded position error,
  temperature/voltage, torque release/hold and stop/disconnection behavior.
- [ ] **Power/load test:** operate drive and both servos together and measure
  supply current, sag/transients and converter limits with external instruments.
  The dashboard does not measure total current or fast power transients.
- [ ] **Failure behavior:** verify browser/network loss, bus failure and
  process interruption; determine controller watchdog capability and recovery.
  Current lease enforcement depends on a running control worker and working I/O.
- [ ] **Pi/network baseline:** verify association, SSH and reboot with the chosen
  topology, actual addresses/ports, permissions and startup commands.

### Next: complete team integration

- [ ] **Vision deployment agreement:** settle direct-to-robot versus laptop
  planner forwarding, one/two-robot delivery, subnet and clock synchronization.
- [ ] **Physical vision acceptance:** calibrate cameras and markers, measure
  pose error/latency/dropouts/skew, then test short camera-corrected moves and
  discrepancy recovery with the actual base. Define acceptable encoder-only duration.
- [ ] **UI handoff:** obtain the teammate repository URL and implementation
  status; agree on hosting, schema/versioning, control ownership and visible
  source/frame/fault/lease state; test preview/Apply, Stop and reconnect in simulation.
  See [UI integration handoff](ui-integration.md).
- [ ] **Listener behavior and five-axis control:** define speaker goal geometry
  and behavior before consuming upstream listener position/height/confidence;
  add aim computation, joint limits, coordinated goals and lost-listener handling.
- [ ] **Proposal behavior and audio interface:** assign the listening-triangle
  solver and Idle/Repositioning/Tracking executive; define obstacle handling,
  speaker/base/joint transforms, target completion and joint-limit/fault feedback
  to the audio/central service. Reconcile the proposed ROS 2 boundary with the
  implemented HTTP/UDP transport and demonstrate the agreed workflow.
- [ ] **Robot camera metadata:** if required, connect upstream `CameraSender`
  to current pan/tilt telemetry and define angle frame, units and sample timing.
- [ ] **Two-robot operation:** assign identity and control authority, distribute
  poses/targets, and validate shared-room behavior. Current local navigation
  neither avoids obstacles/people nor coordinates collision-free robot paths.

### Maintenance after the baseline

- [ ] Add automated CI for Python, JavaScript and lockfile checks; currently no
  repository CI workflow is present.
- [ ] Formalize the UI command/state API and compatibility policy once the
  teammate requirements are known; avoid assuming the internal endpoints are stable.
- [ ] Record reproducible deployment/service startup and diagnostic collection,
  and add real-world acceptance logs tied to commit/configuration versions.
- [ ] Revisit probabilistic fusion or path planning as measured integration
  needs require them. Decide whether ROS is a required proposal deliverable;
  none of these is a dependency of the current code.

## Review verification

- Python suite: **472 passed**, including the two new default-demo rejection
  cases, HTTP servers, UDP reception, recovery and simulated control workers.
- JavaScript: both Node test files passed; all three dashboard entry scripts
  passed syntax checks.
- Ruff lint and formatting checks passed; dependency lock validation passed
  offline; tracked diff whitespace checks passed.
- The current upstream `make_message`/`UdpPublisher` sent a real loopback packet
  accepted by `VisionReceiver`, including populated listener data that was ignored.
- Initial sandbox execution reported 442 passes and 28 failures/errors caused
  by blocked socket creation. The full suite passed with localhost socket access;
  those restrictions were environmental, not failed protocol assertions.

Checks use simulation, mocks and loopback traffic. The complete upstream vision
test suite, real cameras/inference, Pi performance, wireless delivery and robot
hardware were not exercised. No package build/install or external UI validation
was performed. No new tracking package was needed or installed. The only download
was the review clone at `/tmp/capstone-vision-review-20261112`; it is not a
deployment or permanent dependency.

## Documents and Obsidian context

Existing repository guides were retained and corrected where stale. The old
servo implementation plan is explicitly historical. The reviewed Obsidian vision
and target-control notes contain design proposals that now overlap implemented
code; current updates link those historical notes to `Repository Current State`.
The personal planning/procurement notes were left intact.

Maintain this document and its Obsidian mirror as a dated snapshot; use Git
history for completed changes and the checklist above for remaining work. The
[documentation index](README.md) records the maintenance/check workflow.
