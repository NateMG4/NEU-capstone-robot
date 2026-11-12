"use strict";
import { controlState, dialPoint, dialArc, dialModel } from "./controls.mjs";
const $ = (id) => document.getElementById(id);
const clientId = crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random()}`;
let state = null;
let online = false;
let polling = false;
let sending = false;
let samples = [];
let lastEvent = "";
let displayedFault = null;
let durationInitialized = false;
let dialFrameExplicit = false;
const lastTargets = {};
const fmt = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : "—";
const error = (message) => { $("error").textContent = message || ""; $("error").hidden = !message; };

async function send(action, values = {}, quiet = false) {
  if (!state || !online) { if (!quiet) error("Server unavailable. Check the connection before sending commands."); return; }
  try {
    const response = await fetch("./api/command", {
      method: "POST", headers: {"Content-Type": "application/json", "X-Control-Token": state.control_token},
      body: JSON.stringify({action, client_id: clientId, ...values}), signal: AbortSignal.timeout(3500),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "Command rejected");
    if (!quiet) error("");
    return true;
  } catch (err) { if (!quiet) error(err.message); return false; }
}

async function command(action, values = {}) {
  if (sending && !["stop", "torque"].includes(action)) return;
  sending = true;
  render();
  const accepted = await send(action, values);
  if (accepted && (action === "neutral" || action === "move")) {
    for (const [name, field] of [["pan", "yaw_deg"], ["tilt", "pitch_deg"]]) {
      const config = state.axes[name].config;
      const target = action === "neutral" ? 0 : values[field];
      if (state.axes[name].enabled && Number.isFinite(config.zero_deg) && Number.isFinite(target)) lastTargets[name] = config.zero_deg + config.direction * target;
    }
  } else if (accepted && (action === "stop" || action === "recover")) {
    for (const name of Object.keys(lastTargets)) delete lastTargets[name];
  }
  await poll();
  sending = false;
  render();
}

function preview() {
  if (state) renderDials();
}
$("dial-frame").addEventListener("change", () => { dialFrameExplicit = true; preview(); });
for (const id of ["yaw", "pitch"]) {
  $(id).addEventListener("input", () => { $(`${id}-range`).value = $(id).value; preview(); });
  $(`${id}-range`).addEventListener("input", () => { $(id).value = $(`${id}-range`).value; preview(); });
}

$("target-form").addEventListener("submit", (event) => {
  event.preventDefault();
  if (!state || state.test_mode) return;
  const targets = {};
  if (state.axes.pan.enabled) targets.yaw_deg = Number($("yaw").value);
  if (state.axes.tilt.enabled) targets.pitch_deg = Number($("pitch").value);
  command("move", {...targets, duration_s: Number($("duration").value)});
});
$("stop").addEventListener("click", () => command("stop"));
document.addEventListener("keydown", (event) => { if (event.key === "Escape") { event.preventDefault(); command("stop"); } });
$("neutral").addEventListener("click", () => {
  if (!$("duration").reportValidity()) return;
  for (const [name, id] of [["pan", "yaw"], ["tilt", "pitch"]]) if (state.axes[name].enabled) { $(id).value = 0; $(`${id}-range`).value = 0; }
  preview(); command("neutral", {duration_s: Number($("duration").value)});
});
$("use-current").addEventListener("click", () => {
  for (const [name, id] of [["pan", "yaw"], ["tilt", "pitch"]]) {
    if (!state.axes[name].enabled) continue;
    $(id).value = fmt(state.axes[name].joint_deg, 2);
    $(`${id}-range`).value = $(id).value;
  }
  preview();
});
for (const button of document.querySelectorAll("[data-jog]")) button.addEventListener("click", () => {
  if (!$("jog-size").reportValidity() || !$("duration").reportValidity()) return;
  command("jog", {axis: button.dataset.jog, delta_deg: Number(button.dataset.sign) * Number($("jog-size").value), duration_s: Number($("duration").value)});
});
$("refresh").addEventListener("click", () => command("recover"));
for (const button of document.querySelectorAll("[data-capture]")) button.addEventListener("click", () => {
  command("capture_calibration", {axis: $("calibration-axis").value, point: button.dataset.capture});
});
$("save-calibration").addEventListener("click", () => command("save_calibration", {
  axis: $("calibration-axis").value, direction: Number($("calibration-direction").value),
}));
$("use-calibration").addEventListener("click", () => command("use_calibration"));
$("calibration-axis").addEventListener("change", () => {
  $("calibration-direction").value = state.calibration[$("calibration-axis").value].points.direction;
  renderCalibration();
});
$("download").addEventListener("click", () => {
  if (!state) return;
  const {control_token, owner, ...diagnostics} = state;
  const blob = new Blob([JSON.stringify({exported_at: new Date().toISOString(), ...diagnostics, position_history: samples}, null, 2)], {type: "application/json"});
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url; link.download = `pan-tilt-${new Date().toISOString().replaceAll(":", "-")}.json`; link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

function renderTelemetry() {
  for (const name of ["pan", "tilt"]) {
    const data = state.axes[name], status = data.status, config = data.config;
    let row = $(`row-${name}`);
    if (!row) {
      row = document.createElement("tr"); row.id = `row-${name}`;
      for (let i = 0; i < 9; i++) row.appendChild(document.createElement("td"));
      row.children[0].textContent = name === "pan" ? "Pan / yaw" : "Tilt / pitch";
      const id = document.createElement("small"); id.textContent = `LX-224 · ID ${config.servo_id}`; row.children[0].appendChild(id);
      const group = document.createElement("div"); group.className = "button-row";
      for (const enabled of [true, false]) {
        const button = document.createElement("button"); button.type = "button"; button.textContent = enabled ? "Enable" : "Release";
        button.dataset.enabled = String(enabled);
        button.setAttribute("aria-label", `${enabled ? "Enable" : "Release"} ${name} torque`);
        button.addEventListener("click", () => command("torque", {axis: name, enabled})); group.appendChild(button);
      }
      row.children[8].appendChild(group); $("servo-rows").appendChild(row);
    }
    const values = [null, `${fmt(data.joint_deg)}°`, status ? `${fmt(status.position_deg)}°` : "—", status ? `${fmt(status.voltage_v, 2)} V` : "—", status ? `${status.temperature_c} °C` : "—", status ? (status.mode === 0 ? "Position" : "Motor") : "—", status ? (status.torque_enabled ? "Holding" : "Released") : "—", !data.enabled ? "Disabled" : data.error ? "Read failed" : `${fmt(data.age_s, 1)} s`];
    for (let i = 1; i < values.length; i++) row.children[i].textContent = values[i];
    row.children[3].className = status && (status.voltage_v < 6 || status.voltage_v > 8.4) ? "warn" : "";
    row.children[4].className = status && status.temperature_c > config.max_temperature_c ? "warn" : "";
    row.children[5].className = status && status.mode !== 0 ? "warn" : "";
    row.children[7].className = data.error || data.age_s > 2 ? "warn" : "";
    for (const button of row.querySelectorAll("button")) {
      const enabled = button.dataset.enabled === "true";
      const limitsRequired = !state.test_mode && config.min_angle_deg === null;
      button.disabled = !data.enabled || !online || !state.available || (enabled && (!state.connected || !state.allow_motion || limitsRequired || state.active || state.pending || state.fault || sending));
      if (status && status.torque_enabled === enabled) button.disabled = true;
    }
  }
}

function renderCalibration() {
  if (!state.calibration) { $("calibration-tools").hidden = true; return; }
  const select = $("calibration-axis"), names = Object.keys(state.calibration);
  if (!select.options.length) {
    for (const name of names) {
      const option = document.createElement("option"); option.value = name;
      option.textContent = `${name === "tilt" ? "Pitch / tilt" : "Yaw / pan"} · ID ${state.axes[name].config.servo_id}`;
      select.appendChild(option);
    }
    $("calibration-direction").value = state.calibration[select.value].points.direction;
  }
  const data = state.calibration[select.value], axis = state.axes[select.value], points = data.points;
  const busy = state.active || state.pending || sending;
  for (const button of document.querySelectorAll("[data-capture]")) button.disabled = !online || busy || !axis.status || axis.age_s >= 2;
  select.disabled = busy; $("calibration-direction").disabled = busy;
  $("calibration-recordings").textContent = `Servo degrees · Neutral: ${fmt(points.zero_deg, 2)} · End A: ${fmt(points.end_a, 2)} · End B: ${fmt(points.end_b, 2)}`;
  $("save-calibration").disabled = !online || busy || !state.calibration_file || !data.candidate;
  $("use-calibration").disabled = !online || busy || !!state.fault || !names.every(name => state.calibration[name].saved);
  $("calibration-status").textContent = !state.calibration_file ? "Start with --config to save calibration." : data.saved ? `Saved to ${state.calibration_file}. Apply saved calibration to enable joint targets with this neutral and travel.` : data.error || "All three positions recorded. Choose the positive direction, then save.";
}

function svgNode(tag, attributes, text) {
  const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
  if (text !== undefined) node.textContent = text;
  return node;
}

function radialLine(group, name, angle, inner, outer, className) {
  const a = dialPoint(name, angle, inner), b = dialPoint(name, angle, outer);
  group.appendChild(svgNode("line", {x1: a.x, y1: a.y, x2: b.x, y2: b.y, class: className}));
}

function renderDials() {
  const selector = $("dial-frame");
  const jointAvailable = !state.test_mode && Object.values(state.axes).filter(a => a.enabled).every(a => Number.isFinite(a.config.zero_deg));
  selector.options[0].disabled = !jointAvailable;
  if (!dialFrameExplicit) selector.value = jointAvailable ? "joint" : "servo";
  if (!jointAvailable) selector.value = "servo";
  $("orientation-help").textContent = selector.value === "joint" ? "Joint frame · neutral is 0°; labels also show absolute servo angles" : "Absolute servo frame · neutral and limits use the servo's internal coordinates";
  for (const [name, input] of [["pan", "yaw"], ["tilt", "pitch"]]) {
    const axis = state.axes[name];
    const faultAxis = state.fault_report?.axes?.[name];
    let requested = null;
    if (Number.isFinite(axis.target_position_deg)) requested = axis.target_position_deg;
    else if (Number.isFinite(axis.target_deg) && Number.isFinite(axis.config.zero_deg)) requested = axis.config.zero_deg + axis.config.direction * axis.target_deg;
    else if (state.fault && faultAxis) requested = faultAxis.goal_position_deg;
    else if (state.fault) requested = lastTargets[name] ?? null;
    const model = dialModel(axis, {frame: selector.value, testMode: state.test_mode,
      preview: $(input).value.trim() === "" ? null : Number($(input).value), appliedRaw: requested,
      recorded: state.calibration?.[name]});
    const group = $(`${name}-annotations`); group.replaceChildren();
    $(`${name}-frame`).textContent = !model.enabled ? "Disabled for this session" : `${model.frame === "joint" ? "JOINT" : "SERVO"} FRAME${model.draft ? " · RECORDED DRAFT" : ""}`;
    for (const [suffix, value] of [["arm", model.current], ["preview", model.preview], ["requested", model.applied]]) {
      const arm = $(`${name}-${suffix}`);
      arm.setAttribute("visibility", Number.isFinite(value) ? "visible" : "hidden");
      if (Number.isFinite(value)) arm.setAttribute("transform", `rotate(${value * (name === "tilt" ? -1 : 1)} 125 108)`);
    }
    $(`${name}-arm`).classList.toggle("outside", !!model.outOfRange);
    $(`${name}-preview`).classList.toggle("outside", !!model.previewOutside);
    $(`${name}-calibration-values`).replaceChildren();
    $(`${name}-position-note`).textContent = "";
    $(`${name}-angle`).replaceChildren(document.createTextNode(fmt(model.current)));
    const unit = document.createElement("small"); unit.textContent = `° ${model.frame === "joint" ? (name === "pan" ? "yaw" : "pitch") : "servo"}`;
    $(`${name}-angle`).appendChild(unit);
    if (!model.enabled) continue;
    if (model.range) group.appendChild(svgNode("path", {d: dialArc(name, ...model.range), class: `configured-arc${model.draft ? " draft" : ""}`}));
    const ticks = model.frame === "joint" ? [-90, 90, 180] : [0, 60, 120, 180, 240];
    for (const angle of ticks) {
      if (Number.isFinite(model.neutral) && Math.abs(((angle - model.neutral + 540) % 360) - 180) < 12) continue;
      radialLine(group, name, angle, 73, 80, "scale-tick");
      const pos = dialPoint(name, angle, 92);
      group.appendChild(svgNode("text", {x: pos.x, y: pos.y + 3}, `${angle}°`));
    }
    if (model.range) for (const [index, label, radius] of [[0, "MIN", 55], [1, "MAX", 66]]) {
      radialLine(group, name, model.range[index], 69, 85, "limit-marker");
      const pos = dialPoint(name, model.range[index], radius);
      group.appendChild(svgNode("text", {x: pos.x, y: pos.y + 3, class: "limit-label"}, label));
    }
    if (Number.isFinite(model.neutral)) {
      radialLine(group, name, model.neutral, 13, 86, "neutral-marker");
      const pos = dialPoint(name, model.neutral, 92);
      group.appendChild(svgNode("text", {x: pos.x, y: pos.y + 3, class: "zero-label"}, `N ${fmt(model.neutral)}°`));
    }
    for (const [label, value, className] of [["MIN", model.range?.[0], "range-value"], ["NEUTRAL", model.neutral, "zero-value"], ["MAX", model.range?.[1], "range-value"]]) {
      const item = document.createElement("span"); item.className = className;
      item.textContent = `${label} ${fmt(value)}°`; $(`${name}-calibration-values`).appendChild(item);
    }
    const notes = [];
    if (Number.isFinite(model.currentRaw)) notes.push(`Actual ${fmt(model.current, 2)}° ${model.frame} = servo ${fmt(model.currentRaw, 2)}°`);
    if (Number.isFinite(model.zeroRaw)) notes.push(`Neutral = servo ${fmt(model.zeroRaw, 2)}°`);
    else notes.push("Neutral unknown · no mechanical travel calibrated");
    if (Number.isFinite(model.previewRaw)) notes.push(`Preview = servo ${fmt(model.previewRaw, 2)}°${model.previewOutside ? " (outside limits)" : ""}`);
    if (Number.isFinite(model.appliedRaw)) notes.push(`Requested = servo ${fmt(model.appliedRaw, 2)}°`);
    if (model.outOfRange) notes.push("Feedback is outside the configured range");
    $(`${name}-position-note`).textContent = notes.join(" · ");
    $(`${name}-dial`).setAttribute("aria-label", `${name} ${model.frame} angle. ${notes.join(". ")}`);
  }
}

function render() {
  if (!state) return;
  const enabled = Object.entries(state.axes).filter(([, a]) => a.enabled);
  const calibrated = enabled.every(([, a]) => a.config.joint_limits_deg);
  const busy = state.active || state.pending || sending;
  const controls = controlState(state, online, sending);
  $("motion-controls").disabled = !controls.canEdit;
  $("apply").disabled = !controls.canMove;
  $("neutral").disabled = !controls.canMove;
  $("use-current").disabled = !controls.canUseCurrent;
  for (const id of ["apply", "neutral", "use-current"]) $(id).hidden = state.test_mode;
  for (const button of document.querySelectorAll("[data-jog]")) button.disabled = !controls.canJog;
  $("motion-blockers").replaceChildren();
  for (const message of controls.blockers) {
    const item = document.createElement("li"); item.textContent = message;
    $("motion-blockers").appendChild(item);
  }
  $("movement-blocked").hidden = controls.canJog;
  $("coordinate-help").textContent = state.test_mode ? `TEST MODE · Relative jogs up to ${state.jog_max_deg}°. Neutral and mechanical limits are not calibrated. Readouts show absolute servo position; check clearance before each jog.` : "Angles are relative to each joint’s calibrated neutral. Editing a target previews it; Apply moves the mechanism.";
  $("orientation-help").textContent = state.test_mode ? "Absolute servo coordinates · neutral orientation is unknown" : "Independent joint views · not a measured world pose";
  $("history-help").textContent = state.test_mode ? "Last 60 seconds · absolute servo position in degrees" : "Last 60 seconds · servo feedback in joint degrees";
  const jogMax = state.jog_max_deg || 5;
  $("jog-size").max = jogMax;
  if (Number($("jog-size").value) > jogMax) $("jog-size").value = jogMax;
  $("backend").textContent = state.backend === "simulation" ? "SIMULATION" : "LIVE SERVOS";
  $("backend").classList.toggle("live", state.backend === "buslinker");
  $("connection").textContent = online ? (state.connected ? "● Bus connected" : "● Bus unavailable") : "● Server offline";
  $("motion-status").textContent = !online ? "Server unavailable; active motion times out" : state.message;
  $("status-dot").className = `status-dot ${state.fault || !online ? "fault" : state.active ? "active" : ""}`;
  $("mode-note").textContent = state.active ? (state.owner === clientId ? "This browser controls the move · keep this tab visible" : "Another browser controls the move · Stop is always available") : state.backend === "simulation" ? "Simulated feedback · no physical movement" : state.allow_motion ? "Live motion enabled · explicit commands only" : "Read-only · restart with --allow-motion to test movement";
  $("refresh").disabled = !online || !state.available || busy;
  $("stop").disabled = !online;
  const single = enabled.length === 1;
  $("session-axes").textContent = single ? `${enabled[0][0] === "pan" ? "Pan" : "Tilt"} only · the other axis is disabled for this session` : "Pan and tilt · both servos required";
  $("apply-label").textContent = single ? `Apply ${enabled[0][0] === "pan" ? "yaw" : "pitch"}` : "Apply yaw & pitch";
  $("stop-label").textContent = single ? `■ STOP ${enabled[0][0].toUpperCase()}` : "■ STOP BOTH";
  const neutralTravel = enabled.map(([, a]) => a.joint_deg).filter(Number.isFinite).map(Math.abs);
  $("neutral").textContent = neutralTravel.length && !state.test_mode ? `Move to neutral · ${fmt(Math.max(...neutralTravel))}° travel` : "Move to neutral (0°)";
  $("neutral").title = "Commands movement to the configured zero pose. To make the current pose your zero without moving, use Record neutral here.";
  const minimum = Math.max(...enabled.map(([, a]) => a.config.min_duration_s));
  const maximum = Math.min(...enabled.map(([, a]) => a.config.max_duration_s));
  $("duration").min = minimum; $("duration").max = maximum;
  if (!durationInitialized) {
    $("duration").value = Math.min(maximum, Math.max(minimum, 0.5));
    durationInitialized = true;
  }
  $("duration").title = `Smaller values move each step faster. Allowed ${minimum}–${maximum} seconds; feedback adds time between steps.`;
  for (const [name, id] of [["pan", "yaw"], ["tilt", "pitch"]]) {
    const data = state.axes[name], limits = data.config.joint_limits_deg;
    $(`${name}-controls`).disabled = !data.enabled;
    $(`${name}-controls`).classList.toggle("inactive-axis", !data.enabled);
    $(`${name}-preview`).setAttribute("visibility", data.enabled && !state.test_mode ? "visible" : "hidden");
    $(id).closest(".angle-input").hidden = state.test_mode;
    $(`${id}-range`).hidden = state.test_mode;
    if (limits) for (const input of [$(id), $(`${id}-range`)]) { input.min = limits[0]; input.max = limits[1]; }
    $(`${name}-limits`).textContent = !data.enabled ? "Disabled for this session" : state.test_mode ? "Test jogs · no mechanical calibration" : limits ? `${fmt(limits[0], 0)}° … ${fmt(limits[1], 0)}°` : "Limits not configured";
    const angle = state.test_mode ? data.status?.position_deg : data.joint_deg;
    $(`${name}-current`).textContent = data.enabled ? `${state.test_mode ? "Servo" : "Actual"} ${fmt(angle)}°` : "Not in use";
  }
  $("motion-help").textContent = state.test_mode ? "Each jog starts from fresh feedback and checks position afterward. The servo's 0–240° command range is a protocol bound, not measured clearance. Fault recovery reads status and leaves torque changes to you." : !calibrated ? "Preview inputs remain editable. Travel limits must be configured in the server's JSON file before movement is available." : `Moves use steps of at most ${enabled.map(([name, a]) => `${a.config.max_step_deg}° ${name}`).join(" / ")}, with feedback checks and 0.15 s settling per step. Neutral is calibrated zero, not automatic homing.`;
  $("bus-info").textContent = `${state.backend === "simulation" ? "Virtual bus" : enabled[0][1].config.port} · 115200 baud · ${enabled.map(([name, a]) => `${name} (ID ${a.config.servo_id})`).join(" → ")}`;
  $("calibration").textContent = JSON.stringify(Object.fromEntries(Object.entries(state.axes).map(([name, a]) => [name, {enabled: a.enabled, ...a.config}])), null, 2);
  if (state.fault) error(state.fault);
  else if (displayedFault) error("");
  displayedFault = state.fault;
  const eventKey = JSON.stringify(state.events);
  if (eventKey !== lastEvent) {
    lastEvent = eventKey; $("activity").replaceChildren();
    for (const event of state.events) {
      const li = document.createElement("li"), time = document.createElement("time"), message = document.createElement("span");
      time.textContent = event.time; message.textContent = event.message; li.append(time, message); $("activity").appendChild(li);
    }
  }
  renderTelemetry(); renderCalibration(); preview(); drawHistory();
}

function drawHistory() {
  const canvas = $("history"), rect = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio); canvas.height = Math.round(rect.height * ratio);
  const ctx = canvas.getContext("2d"); ctx.scale(ratio, ratio);
  const width = rect.width, height = rect.height, left = 36, top = 8, right = width - 10, bottom = height - 24;
  const now = Date.now(), range = Math.max(10, ...samples.flatMap(s => [Math.abs(s.yaw || 0), Math.abs(s.pitch || 0)]));
  const y = v => bottom - (v + range) / (2 * range) * (bottom - top);
  ctx.font = "9px ui-monospace, monospace";
  for (const value of [-range, 0, range]) {
    ctx.strokeStyle = "#e4ebe3"; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(left, y(value)); ctx.lineTo(right, y(value)); ctx.stroke();
    ctx.fillStyle = "#8b9b91"; ctx.fillText(`${value.toFixed(0)}°`, 0, y(value) + 3);
  }
  ctx.fillText("−60 s", left, height - 3); ctx.fillText("now", right - 18, height - 3);
  for (const [key, color] of [["yaw", "#157d73"], ["pitch", "#ab6e2f"]]) {
    ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 1.8;
    let last = null;
    for (const sample of samples) {
      if (!Number.isFinite(sample[key])) { last = null; continue; }
      const x = left + (sample.time - (now - 60000)) / 60000 * (right - left);
      if (x < left) continue;
      if (last === null || sample.time - last > 1500) ctx.moveTo(x, y(sample[key])); else ctx.lineTo(x, y(sample[key]));
      last = sample.time;
    }
    ctx.stroke();
  }
}

async function poll() {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch("./api/state", {cache: "no-store", signal: AbortSignal.timeout(3500)});
    if (!response.ok) throw new Error("Status request failed");
    state = await response.json(); online = true;
    for (const [name, axis] of Object.entries(state.axes)) {
      if (Number.isFinite(axis.target_position_deg)) lastTargets[name] = axis.target_position_deg;
      else if (Number.isFinite(axis.target_deg) && Number.isFinite(axis.config.zero_deg)) lastTargets[name] = axis.config.zero_deg + axis.config.direction * axis.target_deg;
    }
    const now = Date.now();
    const sample = name => state.axes[name].enabled && !state.axes[name].error && state.axes[name].age_s < 2 ? (state.test_mode ? state.axes[name].status?.position_deg : state.axes[name].joint_deg) : null;
    samples.push({time: now, yaw: sample("pan"), pitch: sample("tilt")});
    samples = samples.filter(sample => now - sample.time <= 60000);
  } catch (_) { online = false; }
  finally { polling = false; render(); }
}

// Hidden or closed controlling tabs request stop; the server heartbeat is the fallback.
function stopOnLeave() {
  if (!state || state.owner !== clientId) return;
  fetch("./api/command", {method: "POST", keepalive: true, headers: {"Content-Type": "application/json", "X-Control-Token": state.control_token}, body: JSON.stringify({action: "stop", client_id: clientId})}).catch(() => {});
}
document.addEventListener("visibilitychange", () => { if (document.hidden) stopOnLeave(); });
window.addEventListener("pagehide", stopOnLeave);
window.addEventListener("resize", drawHistory);
setInterval(() => { if (!document.hidden) { send("heartbeat", {}, true); poll(); } }, 400);
poll(); preview();
