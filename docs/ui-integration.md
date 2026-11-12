# UI integration handoff

Reviewed 2026-11-12. The teammate's UI repository URL, implementation and API
expectations are unknown. No external UI repository is connected, imported or
reviewed. The browser pages in this repository are working robot-control
dashboards; their existence does not establish integration with that future UI.

## Existing system boundary

`capstone-control` runs wheel/navigation and pan/tilt workers and serves their
pages on port 8080. Only those workers own I2C and serial access. HTTP handlers
validate intent and change state or queue work; the frontend should not open
hardware or run another owner of the same motor/servo bus.

For frontend development, run this command from the repository root:

```bash
uv run capstone-control
```

Both systems use simulation by default. Open `http://127.0.0.1:8080`.
Standalone wheel and servo servers expose the same subsystem endpoints on ports
8081 and 8082, with `/api/...` at the root instead of the combined-server prefix.

## Current HTTP contract

| Method | Combined-server path | Responsibility |
| --- | --- | --- |
| GET | `/api/state` | Hub status, subsystem summaries and hub `control_token` |
| POST | `/api/command` | Hub `stop` or `switch` command |
| GET | `/wheels/api/state` | Pose, frame, target, encoders, navigation limits, camera and wheel token |
| POST | `/wheels/api/command` | Wheel/navigation commands |
| GET | `/pan-tilt/api/state` | Axis configuration/feedback, calibration, fault, pending/active and servo token |
| POST | `/pan-tilt/api/command` | Pan/tilt commands |

Requests must use JSON with `Content-Type: application/json`, the matching
state endpoint's `control_token` in `X-Control-Token`, and a `client_id` in the
body. Use a stable ID per controlling browser session, at most 100 characters
to work across all three interfaces. Hub, wheel and servo tokens are distinct
and are regenerated on server restart.

The handlers compare a supplied browser `Origin` against `Host`; they do not
provide cross-origin CORS headers. Serve/proxy the frontend and API under one
origin, or agree on and implement an explicit frontend/backend arrangement.
These process tokens are request checks, not user accounts or network access
control. Deployment currently assumes a trusted lab network or SSH tunnel.

Successful commands return `200 {"ok": true}`. This means intent was accepted;
it does not prove a queued action executed or physical motion finished. Poll
subsystem status and inspect `pending` where provided, `active`, messages,
feedback and faults. Bad tokens/origin yield 403, invalid payloads 400 and
unknown routes 404. Request bodies are limited to 4096 bytes.

## Example payloads

Send each payload to its matching endpoint, using that endpoint's token:

```json
{"action": "target", "client_id": "ui-session-1", "x": 0.2, "y": 0.0, "heading_deg": 0}
```

The wheel `target` action previews and disarms motion. A separate
`{"action":"apply","client_id":"ui-session-1"}` applies the target;
`heartbeat` renews its lease and `stop` disarms it. Targets are within 2 m of
the current estimate. Read `frame_id` before selecting floor/local coordinates.
State poses expose `x`, `y`, `yaw` in radians and `heading_deg`; target commands
take heading in degrees. Hardware requires measured config, the hardware
source and launch motion permission in addition to Apply.

```json
{"action": "jog", "client_id": "ui-session-1", "axis": "pan", "delta_deg": 1, "duration_s": 1}
```

Servo actions include `jog`, `move` (`yaw_deg`/`pitch_deg`), `neutral`, `stop`,
`torque` (`enabled` boolean), `recover`, `capture_calibration`,
`save_calibration` and `use_calibration`. Selected axes, joint limits, torque,
feedback health and test mode constrain which commands are allowed. Test mode
accepts relative jogs up to 2 degrees and rejects absolute targets/neutral.
The existing browser code and Python tests define detailed validation behavior.

```json
{"action": "stop", "client_id": "ui-session-1"}
```

On the hub endpoint, Stop requests stops on both workers while preserving
servo torque. `switch` takes `from` and `to` as `wheels`, `pan-tilt` or
`combined`; leaving combined requests stops on both workers. No switch applies
a target. Treat HTTP stop acceptance as a request and verify subsequent state.

## Interaction requirements

- Renew wheel motion more often than its 0.75-second lease and servo motion
  more often than its 2.5-second heartbeat expiry. The bundled pages currently
  use 200 ms and 400 ms heartbeat intervals respectively. Use the same owner
  `client_id` for renewals; viewing status does not renew movement.
- Keep Stop accessible. Wheel source, target, navigation tuning and camera
  alignment changes stop/disarm movement; require explicit Apply afterward.
  Never automatically reapply after connection loss, fault recovery or restart.
- Show simulation/hardware source, motion permission, frame and feedback age.
  Camera loss retains encoder navigation in the last aligned frame; it does
  not automatically stop a still-leased move. Show that fallback clearly.
- Handle disabled pan/tilt axes, unknown calibration and torque release.
  Servo Stop holds torque; releasing torque is a separate explicit command.
- The two workers have independent goals and leases. The combined view supports
  simultaneous manual operation, not an atomic five-axis trajectory.

## Decisions still needed with the UI teammate

1. Repository URL, owner, framework, supported browser and local run command.
2. Same-origin hosting/proxy, deployed host/port and control authority.
3. Stable versioned request/state schema; current endpoints are an internal
   dashboard interface without a formal API version or published schema.
4. Whether the UI selects fixed targets, listener-relative targets or automatic
   modes, and which service owns planning and listener selection.
5. Reconnect, stale telemetry, multi-browser control and stop acknowledgement UX.
6. Integration tests for preview/Apply, lease expiry, fault recovery, disabled
   axes, camera fallback, tab/page departure and server restart, first in simulation.

Do not infer these decisions or the teammate's progress from this repository.
Track the handoff in [the current-state checklist](current-state.md#remaining-features-and-changes).
