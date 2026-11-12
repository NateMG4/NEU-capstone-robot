// Preview editing and actuator permission are separate. The server also validates every move.
export function controlState(state, online, sending = false) {
  const axes = Object.entries(state.axes).filter(([, axis]) => axis.enabled);
  const blockers = [];
  if (!online) blockers.push("Server unavailable. Restore the dashboard connection.");
  if (!state.available) blockers.push("Serial connection unavailable. Check the server terminal.");
  if (!state.allow_motion) blockers.push("Motion permission is off. Restart with --allow-motion.");
  if (state.fault) blockers.push(`Fault: ${state.fault}. Resolve it, then use Read status / clear fault.`);
  const busy = state.active || state.pending || sending;
  if (busy) blockers.push("A command is in progress. Wait for completion or press Stop.");
  let fresh = axes.length > 0;
  for (const [name, axis] of axes) {
    const title = name === "pan" ? "Pan / yaw" : "Tilt / pitch";
    const config = axis.config;
    if (!state.test_mode && !config.joint_limits_deg) {
      blockers.push(`${title}: travel limits are missing. Set ${name}.zero_deg, ${name}.min_angle_deg and ${name}.max_angle_deg in the file passed to --config, then restart. These are absolute servo angles; the target inputs below do not configure limits.`);
    }
    if (!axis.status || axis.error || !Number.isFinite(axis.age_s) || axis.age_s >= 2) {
      fresh = false;
      blockers.push(`${title}: waiting for fresh servo feedback${axis.error ? ` (${axis.error})` : ""}.`);
      continue;
    }
    const status = axis.status;
    if (status.mode !== 0) blockers.push(`${title}: servo must be in position mode.`);
    if (status.voltage_v < 6 || status.voltage_v > 8.4) blockers.push(`${title}: supply ${status.voltage_v} V is outside 6–8.4 V.`);
    if (status.temperature_c > config.max_temperature_c) blockers.push(`${title}: temperature ${status.temperature_c} °C exceeds ${config.max_temperature_c} °C.`);
    if (config.min_angle_deg !== null && (status.position_deg < config.min_angle_deg || status.position_deg > config.max_angle_deg)) {
      blockers.push(`${title}: current servo angle ${status.position_deg}° is outside the configured ${config.min_angle_deg}–${config.max_angle_deg}° travel limits. Check the calibration before moving.`);
    }
    if (!status.torque_enabled) blockers.push(`${title}: torque is released. Support the mechanism and use Enable in Servo telemetry${state.test_mode ? "." : " after configuring limits."}`);
  }
  if (!axes.length) blockers.push("No axes are enabled for this session.");
  return {
    blockers,
    canEdit: online && !busy,
    canJog: blockers.length === 0,
    canMove: blockers.length === 0 && !state.test_mode,
    canUseCurrent: online && !busy && fresh && !state.test_mode,
  };
}

export function dialPoint(name, angle, radius = 77) {
  const radians = (name === "pan" ? angle - 90 : -angle) * Math.PI / 180;
  return {x: 125 + radius * Math.cos(radians), y: 108 + radius * Math.sin(radians)};
}

export function dialArc(name, lower, upper, radius = 77) {
  if (![lower, upper].every(Number.isFinite) || upper <= lower) return "";
  const start = dialPoint(name, lower, radius), end = dialPoint(name, upper, radius);
  return `M ${start.x} ${start.y} A ${radius} ${radius} 0 ${upper - lower > 180 ? 1 : 0} ${name === "pan" ? 1 : 0} ${end.x} ${end.y}`;
}

export function dialModel(axis, {frame = "joint", testMode = false, preview = null, appliedRaw = null, recorded = null} = {}) {
  const active = !testMode && Number.isFinite(axis.config.zero_deg);
  const config = testMode ? recorded?.candidate : axis.config;
  const displayFrame = frame === "joint" && active ? "joint" : "servo";
  if (!axis.enabled) return {enabled: false, frame: displayFrame, range: null, current: null};
  const zero = config?.zero_deg;
  const direction = config?.direction ?? 1;
  const map = raw => !Number.isFinite(raw) ? null : displayFrame === "joint" ? (raw - zero) * direction : raw;
  const range = config && [config.min_angle_deg, config.max_angle_deg].every(Number.isFinite)
    ? [map(config.min_angle_deg), map(config.max_angle_deg)].sort((a, b) => a - b) : null;
  const rawPreview = active && Number.isFinite(preview) ? axis.config.zero_deg + axis.config.direction * preview : null;
  const currentRaw = axis.status?.position_deg ?? null;
  return {
    enabled: true, frame: displayFrame, range, zeroRaw: zero ?? null,
    neutral: map(zero), currentRaw, current: map(currentRaw),
    previewRaw: rawPreview, preview: map(rawPreview), appliedRaw, applied: map(appliedRaw),
    draft: testMode && !!config,
    outOfRange: !!range && Number.isFinite(currentRaw) && (currentRaw < config.min_angle_deg || currentRaw > config.max_angle_deg),
    previewOutside: !!range && Number.isFinite(rawPreview) && (rawPreview < config.min_angle_deg || rawPreview > config.max_angle_deg),
    minimumRaw: config?.min_angle_deg ?? null, maximumRaw: config?.max_angle_deg ?? null,
  };
}
