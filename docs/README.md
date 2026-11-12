# Project documentation

Start with [Current state](current-state.md): reviewed repository state, recent
work, commit recommendations, validation evidence and the remaining-work checklist.

| Guide | Purpose |
| --- | --- |
| [Dashboard runbook](dashboard-runbook.md) | Main and standalone launch commands, ports and configuration |
| [Hiwonder wheels](hiwonder.md) | I2C protocol, calibration and recorded first-wheel test |
| [Keyboard teleop](teleop.md) | Wheel mapping, controls and local input expiry |
| [Servo bring-up](servo.md) | LX-224 UART/USB, IDs, shared bus, status and bounded moves |
| [Pan/tilt dashboard](pan-tilt-dashboard.md) | Selected axes, test mode, calibration and feedback |
| [Odometry dashboard](odometry-dashboard.md) | Encoder pose, target motion, limits and fault recovery |
| [Camera integration](camera-integration.md) | Verified vision interface, alignment, fallback and deployment gaps |
| [UI integration handoff](ui-integration.md) | Existing HTTP endpoints and decisions for the future UI repository |
| [Combined power test](combined-power-test.md) | Simultaneous drive/servo operation and lab measurements |
| [Pi hotspot](pi-hotspot.md) | Recorded bench network and its difference from the vision network |
| [Historical servo plan](servo-driver-plan.md) | Original design rationale; current guides supersede its proposed work |

## Keeping status current

Use Git for change history and the checklist in `current-state.md` for unfinished
work. Update that report when features land, an integration contract changes, or
a lab result supplies new evidence. Record the date, tested commit/configuration,
observed result and remaining limits; distinguish simulation, mocked hardware,
user reports and direct physical tests. Do not infer deployment from passing tests.

Keep commands and fields in the relevant operating guide. Keep local calibration
and credentials out of Git; version example configurations and dependency locks.
When committing, include Python modules, browser assets, tests and required config
templates together. `git diff --stat` omits untracked files, so also inspect
`git status --short --untracked-files=all`.

The Obsidian Capstone folder contains personal notes and historical AI design
reviews. Its `Repository Current State` note mirrors the dated repository report;
refresh that copy when updating the report. Older notes remain historical records.
No new tracking package or service is required for this workflow.

## Repeatable checks

From the repository root, after `uv sync --locked`:

```bash
uv run pytest -q
uv run ruff check .
uv run ruff format --check .
uv lock --check
node --test tests/*.test.mjs
node --check src/capstone_base/web/app.js
node --check src/capstone_base/pan_tilt_web/app.js
node --check src/capstone_base/control_web/app.js
git diff --check
```

Node is only needed for the JavaScript checks; serving the dashboards requires
no Node runtime or frontend build. HTTP/UDP tests need permission to open
localhost sockets. Hardware extras are optional for offline operation.
