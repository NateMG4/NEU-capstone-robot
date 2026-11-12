import assert from "node:assert/strict";
import test from "node:test";
import { controlState, dialPoint, dialArc, dialModel } from "../src/capstone_base/pan_tilt_web/controls.mjs";

function fixture() {
  return {
    available: true, allow_motion: true, active: false, pending: false, fault: null,
    axes: {
      pan: {enabled: true, error: null, age_s: 0.1,
        config: {joint_limits_deg: [-1, 1], zero_deg: 39.1, min_angle_deg: 38.1, max_angle_deg: 40.1, max_temperature_c: 60},
        status: {position_deg: 39.12, voltage_v: 7.4, temperature_c: 25, mode: 0, torque_enabled: true}},
      tilt: {enabled: false, status: null, error: null, age_s: null,
        config: {joint_limits_deg: null, min_angle_deg: null, max_angle_deg: null}},
    },
  };
}

test("a healthy single pan servo can move with tilt absent", () => {
  const result = controlState(fixture(), true);
  assert.equal(result.canMove, true);
  assert.equal(result.canEdit, true);
  assert.deepEqual(result.blockers, []);
});

test("missing limits keep previews editable and explain where to configure them", () => {
  const state = fixture(); state.axes.pan.config.joint_limits_deg = null;
  const result = controlState(state, true);
  assert.equal(result.canEdit, true);
  assert.equal(result.canUseCurrent, true);
  assert.equal(result.canMove, false);
  assert.match(result.blockers.join(" "), /pan.min_angle_deg/);
  assert.match(result.blockers.join(" "), /absolute servo angles/);
});

test("released torque reports the telemetry Enable control without locking preview", () => {
  const state = fixture(); state.axes.pan.status.torque_enabled = false;
  const result = controlState(state, true);
  assert.equal(result.canEdit, true);
  assert.equal(result.canMove, false);
  assert.match(result.blockers.join(" "), /torque is released.*Enable/);
});

test("read-only launch reports the missing flag", () => {
  const state = fixture(); state.allow_motion = false;
  const result = controlState(state, true);
  assert.equal(result.canMove, false);
  assert.equal(result.canEdit, true);
  assert.match(result.blockers.join(" "), /--allow-motion/);
});

test("a current position outside the configured range blocks movement", () => {
  const state = fixture(); state.axes.pan.status.position_deg = 120;
  const result = controlState(state, true);
  assert.equal(result.canMove, false);
  assert.match(result.blockers.join(" "), /120°.*outside.*38.1–40.1/);
});

test("all blockers are reported together", () => {
  const state = fixture();
  state.axes.pan.config.joint_limits_deg = null;
  state.axes.pan.status.torque_enabled = false;
  state.allow_motion = false;
  assert.equal(controlState(state, true).blockers.length, 3);
});

test("stale or failed feedback cannot move or fill current pose", () => {
  for (const change of [{age_s: 3}, {age_s: null}, {status: null}, {error: "Timeout"}]) {
    const state = fixture(); Object.assign(state.axes.pan, change);
    const result = controlState(state, true);
    assert.equal(result.canMove, false);
    assert.equal(result.canUseCurrent, false);
    assert.match(result.blockers.join(" "), /fresh servo feedback/);
  }
});

test("fault, busy, unhealthy and offline states block commands", () => {
  for (const change of [{fault: "Read failed"}, {active: true}, {pending: true}, {available: false}]) {
    assert.equal(controlState({...fixture(), ...change}, true).canMove, false);
  }
  for (const change of [{mode: 1}, {voltage_v: 5}, {voltage_v: 9}, {temperature_c: 70}]) {
    const state = fixture(); Object.assign(state.axes.pan.status, change);
    assert.equal(controlState(state, true).canMove, false);
  }
  assert.equal(controlState(fixture(), false).canMove, false);
  assert.equal(controlState(fixture(), true, true).canMove, false);
});

test("uncalibrated test mode allows small jog controls and hides calibrated commands", () => {
  const state = fixture(); state.test_mode = true;
  state.axes.pan.config = {...state.axes.pan.config, zero_deg: null,
    joint_limits_deg: null, min_angle_deg: 0, max_angle_deg: 240};
  const result = controlState(state, true);
  assert.deepEqual(result.blockers, []);
  assert.equal(result.canJog, true);
  assert.equal(result.canMove, false);
  assert.equal(result.canUseCurrent, false);
});

test("test mode torque release requires explicit Enable, with no calibration blocker", () => {
  const state = fixture(); state.test_mode = true;
  state.axes.pan.config.joint_limits_deg = null;
  state.axes.pan.status.torque_enabled = false;
  const result = controlState(state, true);
  assert.equal(result.canJog, false);
  assert.equal(result.blockers.length, 1);
  assert.match(result.blockers[0], /torque is released.*Enable/);
  assert.doesNotMatch(result.blockers[0], /configuring limits/);
});

function pitch() {
  return {enabled: true, config: {zero_deg: 191.76, direction: 1,
    min_angle_deg: 90.96, max_angle_deg: 224.16}, status: {position_deg: 120.72}};
}
const near = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-7, `${actual} != ${expected}`);

test("pitch dial maps the Pi calibration and current -71.04 degree offset", () => {
  const model = dialModel(pitch(), {frame: "joint", preview: 0, appliedRaw: 191.76});
  near(model.current, -71.04);
  near(model.range[0], -100.8); near(model.range[1], 32.4);
  assert.equal(model.neutral, 0);
  assert.equal(model.preview, 0);
  assert.equal(model.previewRaw, 191.76);
  assert.equal(model.applied, 0);
  assert.equal(model.outOfRange, false);
});

test("servo frame preserves the absolute positions of neutral, limits and target", () => {
  const model = dialModel(pitch(), {frame: "servo", preview: 0, appliedRaw: 191.76});
  assert.deepEqual(model.range, [90.96, 224.16]);
  assert.equal(model.neutral, 191.76);
  assert.equal(model.current, 120.72);
  assert.equal(model.preview, 191.76);
  assert.equal(model.applied, 191.76);
});

test("reversing direction reverses joint limits and preserves servo coordinates", () => {
  const axis = pitch(); axis.config.direction = -1;
  const joint = dialModel(axis, {preview: 10});
  near(joint.range[0], -32.4); near(joint.range[1], 100.8);
  near(joint.current, 71.04);
  assert.equal(joint.previewRaw, 181.76);
  assert.deepEqual(dialModel(axis, {frame: "servo"}).range, [90.96, 224.16]);
});

test("uncalibrated protocol range is not presented as configured mechanical travel", () => {
  const axis = pitch(); axis.config = {zero_deg: null, direction: 1, min_angle_deg: 0, max_angle_deg: 240};
  const model = dialModel(axis, {testMode: true});
  assert.equal(model.frame, "servo");
  assert.equal(model.range, null);
  assert.equal(model.neutral, null);
  assert.equal(model.preview, null);
});

test("recorded draft is separate from an active calibration", () => {
  const axis = pitch(); const candidate = axis.config;
  const model = dialModel(axis, {testMode: true, recorded: {candidate}});
  assert.equal(model.draft, true);
  assert.equal(model.frame, "servo");
  assert.deepEqual(model.range, [90.96, 224.16]);
  assert.equal(model.neutral, 191.76);
});

test("disabled axes, missing feedback and invalid targets are explicit", () => {
  assert.equal(dialModel({...pitch(), enabled: false}).range, null);
  assert.equal(dialModel({...pitch(), status: null}).current, null);
  assert.equal(dialModel(pitch(), {preview: 100}).previewOutside, true);
  const axis = pitch(); axis.status.position_deg = 240;
  assert.equal(dialModel(axis).outOfRange, true);
});

test("pan positive rotation is clockwise and pitch positive rotation is upward", () => {
  near(dialPoint("pan", 0).x, 125); near(dialPoint("pan", 0).y, 31);
  near(dialPoint("pan", 90).x, 202); near(dialPoint("pan", 90).y, 108);
  near(dialPoint("tilt", 0).x, 202); near(dialPoint("tilt", 0).y, 108);
  near(dialPoint("tilt", 90).x, 125); near(dialPoint("tilt", 90).y, 31);
});

test("range arcs select the correct sweep and support travel greater than 180 degrees", () => {
  assert.match(dialArc("pan", 0, 240), /A 77 77 0 1 1 /);
  assert.match(dialArc("tilt", -70, 30), /A 77 77 0 0 0 /);
  assert.equal(dialArc("pan", null, 20), "");
});
