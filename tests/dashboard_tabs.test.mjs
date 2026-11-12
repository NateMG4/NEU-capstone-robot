import test from "node:test";
import assert from "node:assert/strict";
import {DashboardTabs} from "../src/capstone_base/control_web/tabs.mjs";

test("a switch waits for stop acceptance before unloading the old dashboard", async () => {
  const events = [];
  let accept;
  const tabs = new DashboardTabs({
    send: (action, payload) => { events.push({action, payload}); return new Promise(resolve => { accept = resolve; }); },
    display: name => events.push(name),
  });
  await tabs.select("wheels");
  const switching = tabs.select("pan-tilt");
  assert.equal(tabs.selected, "wheels");
  assert.deepEqual(events, ["wheels", {action: "switch", payload: {from: "wheels", to: "pan-tilt"}}]);
  assert.equal(await tabs.select("pan-tilt"), false);
  accept(true);
  assert.equal(await switching, true);
  assert.equal(tabs.selected, "pan-tilt");
  assert.equal(events.at(-1), "pan-tilt");
});

test("failed stop preserves the current dashboard and allows a later retry", async () => {
  let accept = false;
  const displayed = [];
  const tabs = new DashboardTabs({send: async () => accept, display: name => displayed.push(name)});
  await tabs.select("wheels");
  assert.equal(await tabs.select("pan-tilt"), false);
  assert.equal(tabs.selected, "wheels");
  assert.deepEqual(displayed, ["wheels"]);
  accept = true;
  assert.equal(await tabs.select("pan-tilt"), true);
  assert.deepEqual(displayed, ["wheels", "pan-tilt"]);
});

test("initial load, repeated selection, and unknown routes never request movement", async () => {
  const events = [];
  const tabs = new DashboardTabs({send: async action => { events.push(action); return true; }, display: name => events.push(name)});
  assert.equal(await tabs.select("pan-tilt"), true);
  assert.equal(await tabs.select("pan-tilt"), true);
  assert.equal(await tabs.select("unknown"), false);
  assert.deepEqual(events, ["pan-tilt"]);
});

test("combined view uses the same acknowledged stop transition in both directions", async () => {
  const events = [];
  const tabs = new DashboardTabs({
    send: async (action, payload) => { events.push({action, payload}); return true; },
    display: name => events.push(name),
  });
  await tabs.select("wheels");
  await tabs.select("combined");
  await tabs.select("pan-tilt");
  assert.deepEqual(events, [
    "wheels", {action: "switch", payload: {from: "wheels", to: "combined"}},
    "combined", {action: "switch", payload: {from: "combined", to: "pan-tilt"}}, "pan-tilt",
  ]);
});
