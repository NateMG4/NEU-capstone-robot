import {DashboardTabs, dashboardNames} from "./tabs.mjs";

const $ = id => document.getElementById(id);
const clientId = globalThis.crypto?.randomUUID?.() || `hub-${Date.now()}-${Math.random()}`;
let state = null, online = false, polling = false;
const error = message => { $("error").textContent = message; $("error").hidden = !message; };

async function send(action, payload = {}) {
  if (!online || !state) { error("Waiting for the main dashboard connection."); return false; }
  try {
    const response = await fetch("./api/command", {
      method: "POST", headers: {"Content-Type": "application/json", "X-Control-Token": state.control_token},
      body: JSON.stringify({action, client_id: clientId, ...payload}), signal: AbortSignal.timeout(2500),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "Command failed");
    error("");
    return true;
  } catch (e) { error(e.message); return false; }
}

const tabs = new DashboardTabs({
  send,
  display(name, previous) {
    const combined = name === "combined";
    $("combined-help").hidden = !combined;
    $("dashboard-panel").classList.toggle("combined", combined);
    $("dashboard-frame").src = state.dashboards[combined ? "wheels" : name].path;
    $("dashboard-frame").title = state.dashboards[combined ? "wheels" : name].label;
    $("servo-frame").hidden = !combined;
    $("servo-frame").src = combined ? state.dashboards["pan-tilt"].path : "about:blank";
    $("dashboard-panel").setAttribute("aria-labelledby", `tab-${name}`);
    for (const id of dashboardNames) {
      $(`tab-${id}`).setAttribute("aria-selected", String(id === name));
      $(`tab-${id}`).tabIndex = id === name ? 0 : -1;
    }
    history.replaceState(null, "", `#${name}`);
    $("action-status").textContent = previous ? `Stop requested for ${previous === "combined" ? "wheels and pan/tilt" : state.dashboards[previous].label}. Apply movement explicitly in this view.` : "Both control loops run locally. No movement is applied on page load.";
  },
});

for (const [index, name] of dashboardNames.entries()) {
  $(`tab-${name}`).addEventListener("click", () => tabs.select(name));
  $(`tab-${name}`).addEventListener("keydown", event => {
    if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
    event.preventDefault();
    const next = event.key === "Home" ? 0 : event.key === "End" ? dashboardNames.length - 1 : (index + (event.key === "ArrowLeft" ? dashboardNames.length - 1 : 1)) % dashboardNames.length;
    $(`tab-${dashboardNames[next]}`).focus();
    tabs.select(dashboardNames[next]);
  });
}

$("stop-all").addEventListener("click", async () => {
  if (await send("stop")) $("action-status").textContent = "Stop requested for wheels and pan/tilt. Torque is preserved.";
  await poll();
});
window.addEventListener("hashchange", () => tabs.select(location.hash.slice(1)));
// Escape in the header stops both; a focused child retains its own stop keys.
document.addEventListener("keydown", event => { if (event.key === "Escape") $("stop-all").click(); });

async function poll() {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch("./api/state", {cache: "no-store", signal: AbortSignal.timeout(2500)});
    if (!response.ok) throw new Error("Status unavailable");
    state = await response.json(); online = true;
    $("connection").textContent = "● Connected";
    $("stop-all").disabled = false;
    for (const name of dashboardNames) {
      if (name === "combined") {
        $("combined-status").textContent = "Wheels + pan/tilt · simultaneous control";
        continue;
      }
      const s = state.dashboards[name];
      $(`${name}-status`).textContent = `${s.mode === "simulation" ? "Simulation" : s.mode === "hardware" ? "Live encoders" : "Live servos"} · ${s.fault ? "Fault" : s.active ? "Moving" : "Stopped"}${name === "pan-tilt" && s.test_mode ? " · test mode" : ""}`;
      $(`${name}-status`).title = s.fault || s.message;
    }
    if (!tabs.selected) await tabs.select(dashboardNames.includes(location.hash.slice(1)) ? location.hash.slice(1) : "wheels");
  } catch (_) {
    online = false;
    $("connection").textContent = "○ Disconnected";
    $("stop-all").disabled = true;
    for (const name of dashboardNames) $(`${name}-status`).textContent = "Status unavailable";
    error("Main server unavailable. Reconnect before switching views or issuing commands.");
  } finally { polling = false; }
}

setInterval(poll, 1000);
poll();
