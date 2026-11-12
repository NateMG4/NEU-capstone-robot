"use strict";
const $ = id => document.getElementById(id);
const clientId = (globalThis.crypto && crypto.randomUUID) ? crypto.randomUUID() : `${Date.now()}-${Math.random()}`;
let state = null, token = "", busy = false, connected = false, lastTargetKey = "";
let navigationDirty = false, lastNavigationKey = "";
let displayedFault = null;
const navigationInputs = [...document.querySelectorAll("[data-setting]")];
const navigationMessage = "Settings last for this server session and apply to both sources. Updating stops movement; apply the target again to resume.";
const displayNavigation = (name, value) => name === "yaw_tolerance_rad" ? value * 180 / Math.PI : value;
const internalNavigation = (name, value) => name === "yaw_tolerance_rad" ? value * Math.PI / 180 : value;
const view = {x: 0, y: 0, span: 4};
const canvas = $("map"), ctx = canvas.getContext("2d");
const number = (n, places = 3) => Number(n).toFixed(places);
const metric = (id, n, unit, places = 3) => { $(id).replaceChildren(document.createTextNode(number(n, places) + " "), Object.assign(document.createElement("small"), {textContent: unit})); };
function error(message) { $("error").textContent = message; $("error").hidden = !message; }
async function command(action, payload = {}, quiet = false) {
  try {
    if (!token) throw new Error("Waiting for the server connection");
    const response = await fetch("./api/command", {method: "POST", headers: {"Content-Type": "application/json", "X-Control-Token": token}, body: JSON.stringify({action, client_id: clientId, ...payload}), signal: AbortSignal.timeout(2000)});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "Command failed");
    if (!quiet) error("");
    return true;
  } catch (e) { if (!quiet) error(e.message); return false; }
}
async function poll() {
  if (busy) return;
  busy = true;
  try {
    const response = await fetch("./api/state", {cache: "no-store", signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw new Error("State request failed");
    state = await response.json(); token = state.control_token; connected = true;
    $("connection").textContent = "● Connected · 20 Hz control";
    render();
  } catch (_) {
    connected = false; $("connection").textContent = "○ Disconnected";
    $("apply").checked = false; $("apply").disabled = true;
    $("navigation-fields").disabled = true;
    $("recover").disabled = true;
    $("camera-align").disabled = true; $("camera-disable").disabled = true;
    $("camera-summary").textContent = "Camera telemetry unavailable";
    $("motion-state").textContent = "Connection lost — movement lease will expire";
    error("Server unavailable. Displayed telemetry is stale; reconnect before applying a target.");
  } finally { busy = false; }
}
function render() {
  const s = state, live = s.mode === "hardware";
  const targetKey = JSON.stringify(s.target);
  if (targetKey !== lastTargetKey && !["target-x", "target-y", "target-heading"].includes(document.activeElement.id)) {
    $("target-x").value = number(s.target.x, 2); $("target-y").value = number(s.target.y, 2);
    $("target-heading").value = number(s.target.heading_deg, 1); lastTargetKey = targetKey;
  }
  $("source-badge").textContent = live ? "LIVE ENCODERS" : "SIMULATION";
  $("source-badge").classList.toggle("live", live);
  $("simulation-mode").classList.toggle("selected", !live);
  $("hardware-mode").classList.toggle("selected", live);
  $("hardware-mode").classList.toggle("hardware", live);
  $("hardware-mode").disabled = !s.hardware_available;
  $("hardware-mode").title = s.hardware_available ? "Read the connected motor board" : "Launch with --backend hiwonder and measured config to enable";
  $("source-help").textContent = live ? "Live encoder counts. Pose can drift with wheel slip." : "Ideal simulated wheel counts; models command quantization, not traction or inertia.";
  $("apply-help").textContent = live ? (s.hardware_motion_allowed ? "On: drive real wheels to the target" : "Read-only launch: hardware Apply disabled") : "On: run the simulated robot toward the target";
  $("apply").checked = s.active;
  $("apply").disabled = !connected || !!s.fault || !!s.pose_uncertain || (live && !s.hardware_motion_allowed);
  $("reset").disabled = s.active || !!s.fault;
  $("navigation-fields").disabled = !connected || !!s.fault;
  const navigationKey = JSON.stringify(s.limits);
  if (!navigationDirty && navigationKey !== lastNavigationKey) {
    navigationInputs.forEach(input => {
      const name = input.dataset.setting, bounds = s.navigation_bounds[name];
      input.min = Number(displayNavigation(name, bounds.min).toPrecision(12)); input.max = Number(displayNavigation(name, bounds.max).toPrecision(12));
      input.value = Number(displayNavigation(name, s.limits[name]).toPrecision(12));
    });
    for (const [slider, name] of [["speed-slider", "max_speed_m_s"], ["yaw-slider", "max_yaw_rad_s"]]) {
      $(slider).min = s.navigation_bounds[name].min; $(slider).max = s.navigation_bounds[name].max;
      $(slider).value = s.limits[name];
    }
    lastNavigationKey = navigationKey;
  }
  $("speed-ceiling").textContent = `Current wheel limit: ${number(s.limits.max_wheel_rad_s, 3)} rad/s (launch ceiling ${number(s.launch_limits.max_wheel_rad_s, 3)}). Straight-line ceiling ≈ ${number(s.limits.max_wheel_rad_s * s.geometry.wheel_radius_m, 3)} m/s; combining turning and translation may run slower.`;
  $("motion-state").textContent = s.status;
  $("frame-label").textContent = `${s.frame_id === "odom" ? "Local" : "Aligned"} frame · ${s.frame_id}`;
  $("target-frame-help").textContent = `Targets use the ${s.frame_id} frame. Select within 2 m of the estimated robot position.`;
  const camera = s.camera, corrected = camera.enabled && camera.connected && !camera.correction_error && camera.last_applied_epoch !== null;
  const cameraStatus = !camera.configured ? "Camera not configured" : camera.alignment_pending ? "Aligning · movement off" : camera.correction_error ? camera.correction_error : camera.enabled ? (corrected ? "Camera corrections active" : "Camera stale · continuing on encoders") : camera.connected ? "Camera available · corrections off" : "Waiting for fresh camera data";
  $("camera-summary").textContent = cameraStatus;
  $("camera-status").textContent = cameraStatus;
  $("camera-align").disabled = !connected || !!s.fault || !camera.connected || camera.alignment_pending;
  $("camera-disable").disabled = !connected || !!s.fault || (!camera.enabled && !camera.alignment_pending);
  $("camera-pose").textContent = camera.pose ? `Last camera: X ${number(camera.pose.x)} m · Y ${number(camera.pose.y)} m · heading ${number(camera.pose.yaw * 180 / Math.PI, 1)}°` : "Waiting for a robot pose.";
  $("camera-details").textContent = camera.configured ? `Robot ${camera.configuration.robot_id} · frame ${camera.configuration.frame_id} · sighting age ${camera.age_s === null ? "—" : number(camera.age_s * 1000, 0) + " ms"} · packets ${camera.accepted} accepted / ${camera.rejected} rejected.${camera.last_error ? " Last input error: " + camera.last_error : ""}` : "Launch with --vision-config config/vision.example.json to receive UDP positions.";
  $("map-source").textContent = corrected ? "Encoder estimate with capture-time camera corrections" : s.frame_id === "odom" ? "Encoder estimate · local origin" : `Encoder estimate in ${s.frame_id} · last alignment retained`;
  if (s.fault) error(s.fault);
  else if (displayedFault && $("error").textContent === displayedFault) error("");
  displayedFault = s.fault;
  $("recovery-panel").hidden = !s.fault && !s.pose_uncertain;
  $("recover").hidden = !s.fault;
  $("recover").disabled = !connected || !s.recovery_available || s.recovery_pending;
  $("recover").textContent = s.recovery_pending ? "Recovering… movement stays off" : "Recover encoders · stay stopped";
  $("recovery-heading").textContent = s.recovery_pending ? "Re-establishing encoder baseline" : s.fault ? "Encoder fault · movement disabled" : "Recovered · position needs a reference";
  $("recovery-help").textContent = s.fault ? "Recovery sends zero, reinitializes the connected board, and requires stable counters. It keeps the last valid pose, clears the target, and leaves movement off. It does not fix an incorrect scale or wiring." : "Movement during the fault was discarded. Reset the local origin at your chosen starting position, then select a new target. The old room position is unverified.";
  $("pose-confidence").textContent = s.pose_uncertain ? "Position uncertain: motion since the last valid sample was not integrated. Reset the local origin or align a fresh camera pose." : corrected ? "Camera corrects position and heading; wheel encoders estimate movement between observations." : "Encoder estimate; slip since the last alignment is unobserved.";
  const calibration = s.encoder_configuration;
  $("encoder-config").textContent = `Configuration: ${calibration.path || "provided configuration"}. Latest fault source: ${s.fault_report ? s.fault_report.source : "none"}.`;
  $("encoder-calibration").textContent = `Wheel order: FL, FR, RL, RR\nCounts / wheel revolution: ${calibration.counts_per_wheel_rev.join(", ")}\nM channels: ${calibration.wheel_channels.join(", ")}\nEncoder signs: ${calibration.encoder_signs.join(", ")}\nMotor signs: ${calibration.motor_signs.join(", ")}\nMotor type / polarity: ${calibration.motor_type} / ${calibration.encoder_polarity}\nPlausibility guard: ${number(calibration.max_encoder_wheel_rad_s, 2)} rad/s + 2 counts`;
  const details = s.fault_report ? s.fault_report.details : {};
  $("rejected-summary").textContent = s.fault_report ? `${s.fault_report.message}${details.dt_s !== undefined ? ` · sample interval ${number(details.dt_s * 1000, 3)} ms` : ""}. Raw rows below describe the rejected sample, not the frozen live estimate.` : "No rejected sample recorded.";
  $("rejected-rows").replaceChildren(...(details.wheels || []).map(wheel => {
    const row = document.createElement("tr");
    if (wheel.rejected) row.className = "rejected-row";
    const expected = s.fault_report.expected_abs_delta_from_command;
    [`${wheel.wheel} / M${wheel.channel}`, wheel.previous_count, wheel.current_count, wheel.delta_counts, wheel.allowed_delta_counts === null ? "—" : number(wheel.allowed_delta_counts, 1), expected ? number(expected[wheel.channel - 1], 1) : "—", wheel.inferred_rad_s === null ? "—" : number(wheel.inferred_rad_s), wheel.counts_per_wheel_rev].forEach(value => {
      const td = document.createElement("td"); td.textContent = value; row.append(td);
    }); return row;
  }));
  metric("pose-x", s.pose.x, "m"); metric("pose-y", s.pose.y, "m"); metric("pose-heading", s.pose.heading_deg, "°", 1);
  if (s.sample_age_s === null) $("sample-age").textContent = "Waiting"; else metric("sample-age", s.sample_age_s * 1000, "ms", 0);
  metric("distance", s.plan.distance, "m"); metric("heading-error", s.plan.yaw_error * 180 / Math.PI, "°", 1);
  ["vx", "vy", "wz"].forEach((name, i) => { $("cmd-" + name).textContent = number(s.plan.body[i]); });
  $("wheel-limit").textContent = `LIMIT ${number(s.limits.max_wheel_rad_s, 1)} RAD/S`;
  const rows = ["FL", "FR", "RL", "RR"].map((name, i) => {
    const row = document.createElement("tr");
    [name, s.raw_counts ? s.raw_counts[s.wheel_channels[i] - 1] : "—", s.delta_counts[i], number(s.wheel_distances_m[i]), number(s.wheel_rad_s[i]), number(s.plan.wheels[i]), number(s.applied_wheels[i])].forEach((value, j) => {
      const td = document.createElement("td"); td.textContent = value;
      if (j === 0) td.append(Object.assign(document.createElement("small"), {textContent: `M${s.wheel_channels[i]}`}));
      if (j === 5) td.className = "planned"; if (j === 6) td.className = "applied";
      row.append(td);
    }); return row;
  });
  $("wheel-rows").replaceChildren(...rows);
  $("raw-counts").textContent = `M1…M4: ${s.raw_counts ? s.raw_counts.join(", ") : "waiting for baseline"}`;
  $("count-commands").textContent = `M1…M4 counts/10ms · planned [${s.planned_motor_counts.join(", ")}] · applied [${s.applied_motor_counts.join(", ")}]`;
  draw();
}
function sizing() {
  const rect = canvas.getBoundingClientRect(), scale = Math.min(rect.width, rect.height) / view.span;
  return {w: rect.width, h: rect.height, scale, point: (x, y) => [rect.width / 2 + (x - view.x) * scale, rect.height / 2 - (y - view.y) * scale]};
}
function line(a, b, color, width = 1, dash = []) { ctx.beginPath(); ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); ctx.setLineDash([]); }
function arrow(a, b, color, width = 2) {
  if (Math.hypot(b[0]-a[0], b[1]-a[1]) < 1) return;
  line(a, b, color, width); const angle = Math.atan2(b[1]-a[1], b[0]-a[0]);
  ctx.beginPath(); ctx.fillStyle = color; ctx.moveTo(...b);
  ctx.lineTo(b[0]-7*Math.cos(angle-.45), b[1]-7*Math.sin(angle-.45));
  ctx.lineTo(b[0]-7*Math.cos(angle+.45), b[1]-7*Math.sin(angle+.45)); ctx.closePath(); ctx.fill();
}
function draw() {
  const {w, h, scale, point} = sizing(), dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(w*dpr); canvas.height = Math.round(h*dpr); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
  const step = view.span <= 3 ? .25 : view.span <= 7 ? .5 : view.span <= 14 ? 1 : 2;
  const minX = view.x-w/scale/2, maxX = view.x+w/scale/2, minY = view.y-h/scale/2, maxY = view.y+h/scale/2;
  ctx.font = "10px ui-monospace, monospace"; ctx.fillStyle = "#59718f";
  for (let x = Math.ceil(minX/step)*step; x <= maxX; x += step) { line(point(x,minY),point(x,maxY), Math.abs(x)<1e-7 ? "#344b64" : "#192b40"); if (Math.abs(x)>1e-7) ctx.fillText(number(x,2).replace(/0+$/, "").replace(/\.$/, ""), point(x,view.y)[0]+4, h/2+13); }
  for (let y = Math.ceil(minY/step)*step; y <= maxY; y += step) { line(point(minX,y),point(maxX,y), Math.abs(y)<1e-7 ? "#344b64" : "#192b40"); if (Math.abs(y)>1e-7) ctx.fillText(number(y,2).replace(/0+$/, "").replace(/\.$/, ""), w/2+5, point(view.x,y)[1]-5); }
  const origin = point(0,0); ctx.fillStyle = "#8094ae"; ctx.fillText("0",origin[0]+5,origin[1]+13);
  $("map-scale").textContent = `${number(view.span,1)} m short-axis view · ${step} m grid`;
  if (!state) return;
  const s = state, p = s.pose, target = point(s.target.x,s.target.y);
  if (s.trail.length > 1) { ctx.beginPath(); ctx.strokeStyle = "#3c638d"; ctx.lineWidth = 2; s.trail.forEach((p,i) => { const v=point(p.x,p.y); if(i) ctx.lineTo(...v); else ctx.moveTo(...v); }); ctx.stroke(); }
  if (s.camera.connected && s.camera.configuration.frame_id === s.frame_id && s.camera.pose) {
    const cp = s.camera.pose, a = point(cp.x, cp.y);
    ctx.beginPath(); ctx.arc(...a, 8, 0, 2 * Math.PI); ctx.strokeStyle = "#74ebc1"; ctx.lineWidth = 2; ctx.stroke();
    arrow(a, [a[0] + 22 * Math.cos(cp.yaw), a[1] - 22 * Math.sin(cp.yaw)], "#74ebc1", 1.5);
  }
  line(point(s.path[0].x,s.path[0].y),point(s.path[1].x,s.path[1].y),"#b89260",1.5,[6,6]);
  const [tx,ty] = target; ctx.strokeStyle="#f7bf70";ctx.lineWidth=1.5;ctx.strokeRect(tx-7,ty-7,14,14);
  arrow(target, [tx+23*Math.cos(s.target.yaw),ty-23*Math.sin(s.target.yaw)],"#f7bf70",1.5);
  ctx.fillStyle="#f7bf70";ctx.fillText(`TARGET (${number(s.target.x,2)}, ${number(s.target.y,2)})`,tx+13,ty-14);
  const c=Math.cos(p.yaw), sn=Math.sin(p.yaw), bodyPoint=(x,y)=>point(p.x+c*x-sn*y,p.y+sn*x+c*y);
  const halfX=s.geometry.wheelbase_m/2, halfY=s.geometry.track_width_m/2;
  const corners=[[halfX,halfY],[-halfX,halfY],[-halfX,-halfY],[halfX,-halfY]];
  ctx.beginPath();corners.forEach(([x,y],i)=>{const v=bodyPoint(x,y);if(i)ctx.lineTo(...v);else ctx.moveTo(...v);});ctx.closePath();ctx.fillStyle="#162d49";ctx.fill();ctx.strokeStyle="#70adff";ctx.lineWidth=2;ctx.stroke();
  const center=point(p.x,p.y); arrow(center,bodyPoint(halfX+.1,0),"#70adff",2);
  [[halfX,halfY],[halfX,-halfY],[-halfX,halfY],[-halfX,-halfY]].forEach(([x,y],i)=>{
    const a=bodyPoint(x,y),speed=s.plan.wheels[i], length=35*speed/s.limits.max_wheel_rad_s;
    ctx.fillStyle="#70adff";ctx.beginPath();ctx.arc(...a,3,0,2*Math.PI);ctx.fill();
    arrow(a,[a[0]+c*length,a[1]-sn*length],"#9db9e5",1.5);
    ctx.fillStyle="#93a8c3";ctx.fillText(["FL","FR","RL","RR"][i],a[0]+5,a[1]+13);
  });
  const [vx,vy]=s.plan.body, worldX=c*vx-sn*vy, worldY=sn*vx+c*vy;
  arrow(center,[center[0]+worldX*450,center[1]-worldY*450],"#74ebc1",3);
  ctx.fillStyle="#a7c7f5";ctx.fillText("ROBOT",center[0]+12,center[1]-16);
}
$("target-form").addEventListener("submit", async event => { event.preventDefault(); await command("target", {x:Number($("target-x").value),y:Number($("target-y").value),heading_deg:Number($("target-heading").value)}); await poll(); });
function navigationEdited() {
  navigationDirty = true;
  $("navigation-status").textContent = "Unsaved changes. Update navigation to refresh the preview; updating will stop the current move.";
}
navigationInputs.forEach(input => input.addEventListener("input", () => {
  navigationEdited();
  if (input.dataset.setting === "max_speed_m_s") $("speed-slider").value = input.value;
  if (input.dataset.setting === "max_yaw_rad_s") $("yaw-slider").value = input.value;
}));
for (const [slider, name] of [["speed-slider", "max_speed_m_s"], ["yaw-slider", "max_yaw_rad_s"]]) {
  $(slider).addEventListener("input", () => {
    $("nav-" + name).value = Number(Number($(slider).value).toPrecision(8)); navigationEdited();
  });
}
$("navigation-form").addEventListener("submit", async event => {
  event.preventDefault();
  const navigation = Object.fromEntries(navigationInputs.map(input => [input.dataset.setting, internalNavigation(input.dataset.setting, Number(input.value))]));
  if (await command("navigation", {navigation})) {
    navigationDirty = false; lastNavigationKey = "";
    $("navigation-status").textContent = "Navigation updated. " + navigationMessage;
    await poll();
  }
});
$("navigation-reset").addEventListener("click", async () => {
  if (await command("navigation_reset")) {
    navigationDirty = false; lastNavigationKey = "";
    $("navigation-status").textContent = "Launch values restored. " + navigationMessage;
    await poll();
  }
});
canvas.addEventListener("click", async event => {
  if (!state || !connected) return;
  const rect=canvas.getBoundingClientRect(), {w,h,scale}=sizing();
  const x=view.x+(event.clientX-rect.left-w/2)/scale,y=view.y-(event.clientY-rect.top-h/2)/scale;
  $("target-x").value=number(x,2);$("target-y").value=number(y,2);
  await command("target",{x,y,heading_deg:Number($("target-heading").value)});await poll();
});
$("apply").addEventListener("change", async event => { await command(event.target.checked ? "apply" : "stop"); await poll(); });
$("stop").addEventListener("click",async()=>{await command("stop");await poll();});
$("reset").addEventListener("click",async()=>{await command("reset");await poll();});
$("recover").addEventListener("click", async () => { await command("recover"); await poll(); });
$("camera-align").addEventListener("click", async () => { await command("camera_align"); await poll(); });
$("camera-disable").addEventListener("click", async () => { await command("camera_disable"); await poll(); });
$("download-diagnostics").addEventListener("click", () => {
  if (!state) return;
  const report = {exported_at: new Date().toISOString(), mode: state.mode, status: state.status,
    fault: state.fault, fault_report: state.fault_report, recovery_count: state.recovery_count,
    configuration: state.encoder_configuration, pose: state.pose, pose_uncertain: state.pose_uncertain,
    camera: state.camera, encoder_pose: state.encoder_pose, frame_id: state.frame_id,
    raw_counts: state.raw_counts, latest_encoder_read: state.latest_encoder_read, limits: state.limits};
  const url = URL.createObjectURL(new Blob([JSON.stringify(report, null, 2)], {type: "application/json"}));
  const link = document.createElement("a"); link.href = url; link.download = "encoder-diagnostics.json";
  document.body.append(link); link.click(); link.remove(); setTimeout(() => URL.revokeObjectURL(url), 1000);
});
for (const mode of ["simulation","hardware"]) $(mode+"-mode").addEventListener("click",async()=>{await command("mode",{mode});await poll();});
$("zoom-in").addEventListener("click",()=>{view.span=Math.max(.5,view.span/1.3);draw();});
$("zoom-out").addEventListener("click",()=>{view.span=Math.min(40,view.span*1.3);draw();});
$("fit").addEventListener("click",()=>{if(!state)return;const points=[state.pose,state.target,{x:0,y:0},...state.trail];const xs=points.map(p=>p.x),ys=points.map(p=>p.y);view.x=(Math.max(...xs)+Math.min(...xs))/2;view.y=(Math.max(...ys)+Math.min(...ys))/2;view.span=Math.max(2,Math.max(...xs)-Math.min(...xs)+.8,Math.max(...ys)-Math.min(...ys)+.8);draw();});
document.addEventListener("keydown",event=>{if(event.code==="Space"&&!/INPUT|TEXTAREA|BUTTON|SELECT/.test(event.target.tagName)){event.preventDefault();command("stop").then(poll);}});
function abandon() { if(state&&state.active) { command("stop",{},true); state.active=false; } }
document.addEventListener("visibilitychange",()=>{if(document.hidden)abandon();});
window.addEventListener("pagehide",abandon);
window.addEventListener("resize",draw);
setInterval(()=>{if(connected&&state&&state.active&&!document.hidden)command("heartbeat",{},true);},200);
setInterval(poll,200);poll();draw();
