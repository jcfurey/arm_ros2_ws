/* A dependency-free Lichtblick panel. All motion goes through arm_bringup and MoveIt. */
"use strict";

const COMMAND = "/motion/command";
const STATUS = "/motion/status";
const POSE = "/motion/goal_pose";
const JOINTS = "/motion/goal_joints";
// MoveIt's own stop channel. Stop publishes here as well as to the workcell
// UI, so it still halts the arm when the UI has stopped answering.
const EVENT = "/trajectory_execution_event";
const DEG = Math.PI / 180;

function quaternion(r, p, y) {
  const [sr, cr, sp, cp, sy, cy] = [Math.sin(r/2), Math.cos(r/2), Math.sin(p/2), Math.cos(p/2), Math.sin(y/2), Math.cos(y/2)];
  return { x: sr*cp*cy-cr*sp*sy, y: cr*sp*cy+sr*cp*sy, z: cr*cp*sy-sr*sp*cy, w: cr*cp*cy+sr*sp*sy };
}

function rpy([x, y, z, w]) {
  return [Math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)), Math.asin(Math.max(-1, Math.min(1, 2*(w*y-z*x)))),
    Math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))];
}

const STYLES = `
:host { display:block; height:100%; color:#e7edf5; font:13px -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; background:#101720; }
* { box-sizing:border-box; } .shell { height:100%; display:flex; flex-direction:column; } .panel { flex:1; min-height:0; overflow:auto; padding:18px; display:flex; flex-direction:column; gap:15px; }
.top,.row { display:flex; align-items:center; gap:9px; } .top { justify-content:space-between; } h1 { font-size:18px; margin:0; font-weight:650; }
.muted { color:#9aaabd; font-size:12px; line-height:1.5; } .connection { color:#f0b95b; font-size:11px; letter-spacing:.03em; }
label { display:flex; flex-direction:column; gap:6px; color:#b9c8da; font-size:12px; } select,input,button { font:inherit; }
select,input[type=number] { color:#eaf2fd; background:#1d2938; border:1px solid #40506a; border-radius:6px; padding:9px; min-width:0; width:100%; }
select { cursor:pointer; } select:focus,input:focus { outline:2px solid #4ba9ee; outline-offset:1px; }
button { border:1px solid #46556b; background:#243347; color:#eaf2fd; border-radius:6px; padding:10px 13px; cursor:pointer; min-height:38px; }
button:hover:not(:disabled) { filter:brightness(1.18); } button:disabled,select:disabled,input:disabled { opacity:.42; cursor:default; }
button.primary { background:#1680bf; border-color:#2799db; font-weight:650; } button.stop { background:#9f303d; border-color:#ce4b5a; font-weight:650; }
button.secondary { padding:6px 9px; min-height:30px; font-size:12px; } .wide { width:100%; } .row>* { flex:1; } .row>.compact { flex:0 0 auto; }
.mode { display:flex; flex-direction:column; gap:12px; } [hidden] { display:none !important; } .grid { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:9px; }
.section-title { display:flex; align-items:center; justify-content:space-between; gap:8px; font-size:12px; font-weight:650; color:#d1dfef; }
.speed { display:flex; flex-direction:column; gap:7px; } input[type=range] { width:100%; accent-color:#47b4f1; margin:3px 0; }
.joint { display:grid; grid-template-columns:50px 1fr 78px; align-items:center; gap:8px; } .joint label { font-variant-numeric:tabular-nums; } .joint input[type=number] { padding:6px; }
.status { border:1px solid #34475e; border-left:3px solid #7790ae; border-radius:7px; padding:11px; background:#172230; }
.status[data-tone=ok] { border-left-color:#51d4a1; } .status[data-tone=error] { border-left-color:#fa7786; } .status[data-tone=busy] { border-left-color:#53bdfa; }
.status-name { font-weight:650; margin-bottom:5px; } .detail { color:#b3c3d6; font-size:12px; line-height:1.5; white-space:pre-wrap; overflow-wrap:anywhere; }
.metrics { display:grid; grid-template-columns:1fr 1fr; gap:10px; padding-top:10px; margin-top:9px; border-top:1px solid #314257; font-size:11px; color:#a5b7cb; }
.metrics strong { display:block; color:#e2eefb; font-size:14px; font-weight:550; margin-bottom:2px; font-variant-numeric:tabular-nums; }
.actions { flex-shrink:0; display:grid; grid-template-columns:1.4fr 1fr 1fr; gap:8px; padding:12px 18px; background:#101720; border-top:1px solid #34475e; } .tool-readout { color:#9ccbe9; font-size:11px; font-variant-numeric:tabular-nums; line-height:1.5; }
.steps { display:flex; flex-wrap:wrap; gap:5px; } .step { color:#758ca6; padding:4px 6px; font-size:10px; border-radius:4px; background:#182433; } .step.active { color:#8ee2ff; background:#17405a; }
.history { margin-top:auto; border-top:1px solid #29394e; padding-top:10px; } .history summary { cursor:pointer; color:#8fa3bb; font-size:11px; }
.history ol { list-style:none; margin:8px 0 0; padding:0; display:flex; flex-direction:column; gap:7px; color:#94a9c1; font-size:11px; }
.history time { color:#637e9b; margin-right:7px; } .error { color:#ff9aa7; } .helper { margin:0; } @media(max-width:340px) { .panel { padding:12px; } .joint { grid-template-columns:42px 1fr 65px; } }
`;

function initPanel(context) {
  const document = context.panelElement.ownerDocument;
  const root = context.panelElement.attachShadow({ mode: "open" });
  root.innerHTML = `<style>${STYLES}</style><div class="shell"><main class="panel">
    <header class="top"><h1>MoveIt controls</h1><span id="connection" class="connection">Waiting for workcell</span></header>
    <label>Control mode<select id="mode" aria-label="Control mode">
      <option value="named">Named pose</option><option value="cartesian">Cartesian tool pose</option>
      <option value="joints">Joint targets</option><option value="pick">Pick and place</option></select></label>
    <section id="named" class="mode"><label>Arm pose<select id="named-pose" aria-label="Arm pose"></select></label>
      <p class="muted helper">Plan first. The cyan path and translucent robot show the tool path and goal in the 3D view.</p>
      <button id="plan-named" class="primary wide">Plan pose</button></section>
    <section id="cartesian" class="mode" hidden><div class="section-title"><span>Tool target · world frame</span><button id="read-tool" class="secondary">Use current pose</button></div>
      <div class="grid"><label>X (m)<input id="x" type="number" step="0.01" value="0.3"></label><label>Y (m)<input id="y" type="number" step="0.01" value="0"></label><label>Z (m)<input id="z" type="number" step="0.01" value="1.2"></label></div>
      <div class="grid"><label>Roll (°)<input id="roll" type="number" step="1" value="180"></label><label>Pitch (°)<input id="pitch" type="number" step="1" value="0"></label><label>Yaw (°)<input id="yaw" type="number" step="1" value="0"></label></div>
      <p class="muted helper">Collision-checked tool goal. MoveIt chooses the path; it may curve between poses.</p><button id="plan-tool" class="primary wide">Plan tool pose</button></section>
    <section id="joints" class="mode" hidden><div class="section-title"><span>Joint targets · degrees</span><button id="read-joints" class="secondary">Use current joints</button></div>
      <div id="joint-fields" class="mode"></div><button id="plan-joints" class="primary wide">Plan joints</button></section>
    <section id="pick" class="mode" hidden><label>Part<select id="part" aria-label="Part"></select></label>
      <label>Place destination<select id="destination" aria-label="Place destination"><option value="auto">Other location (toggle)</option><option value="drop">Drop zone</option><option value="home">Original position</option></select></label>
      <p class="muted helper">Runs the full approach, grasp, carry and place sequence. The active step appears below.</p>
      <button id="pick-place" class="primary wide">Pick + place</button><div class="row"><button id="open">Open gripper</button><button id="close">Close gripper</button></div>
      <div id="steps" class="steps"></div></section>
    <section class="speed"><div class="section-title"><span>Motion speed</span><span id="speed-label">25%</span></div><input id="speed" aria-label="Motion speed" type="range" min="5" max="100" step="5" value="25">
      <span class="muted">Changing speed clears the preview. Grasp approach/lift stays at 10% or slower.</span></section>
    <section id="status" class="status"><div id="status-name" class="status-name">Waiting for status</div><div id="detail" class="detail">Connect to the running workcell.</div>
      <div class="metrics"><div><strong id="duration">—</strong>planned duration</div><div><strong id="points">—</strong>trajectory waypoints</div></div></section>
    <div id="tool-readout" class="tool-readout">Tool pose unavailable</div><div id="local-error" class="error" role="alert"></div>
    <details class="history"><summary>Recent motion events</summary><ol id="events"></ol></details>
  </main><footer class="actions"><button id="execute" class="primary">Execute plan</button><button id="clear">Clear</button><button id="stop" class="stop">Stop</button></footer></div>`;
  const $ = (id) => root.getElementById(id);
  const state = { status: undefined, received: 0, advertised: false, joints: [], events: [], eventKey: "" };
  const saved = { ...(context.initialState || {}) };
  function persist(patch) { Object.assign(saved, patch); context.saveState({ ...saved }); }
  if (Array.isArray(saved.toolTarget) && saved.toolTarget.length === 6) {
    ["x", "y", "z", "roll", "pitch", "yaw"].forEach((id, index) => { $(id).value = saved.toolTarget[index]; });
  }
  if (["auto", "home", "drop"].includes(saved.destination)) $("destination").value = saved.destination;
  $("mode").value = ["named", "cartesian", "joints", "pick"].includes(saved.mode) ? saved.mode : "named";
  const clock = () => Date.now();
  const fresh = () => state.received > 0 && clock() - state.received < 4000;

  // Stop passes `always`: a stale status must never keep it from being sent.
  function publish(topic, message, always = false) {
    try {
      if (!context.publish || !state.advertised || !(always || fresh())) throw new Error("Waiting for a live workcell connection");
      context.publish(topic, message);
      $("local-error").textContent = "";
    } catch (error) { $("local-error").textContent = error.message || String(error); }
  }
  const command = (data) => publish(COMMAND, { data });
  function options(select, values, preferred) {
    const current = select.value || preferred;
    if (JSON.stringify(Array.from(select.options, (option) => option.value)) !== JSON.stringify(values)) {
      select.replaceChildren(...values.map((value) => { const option = document.createElement("option"); option.value = value; option.textContent = value.replaceAll("_", " "); return option; }));
      if (values.includes(current)) select.value = current;
    }
  }
  function modeChanged() {
    for (const mode of ["named", "cartesian", "joints", "pick"]) $(mode).hidden = mode !== $("mode").value;
    persist({ mode: $("mode").value });
  }
  $("mode").addEventListener("change", modeChanged);
  modeChanged();

  function readTool() {
    const pose = state.status?.tool_pose;
    if (!pose) return;
    ["x", "y", "z"].forEach((id, i) => { $(id).value = pose[i].toFixed(3); });
    rpy(pose.slice(3)).forEach((angle, i) => { $(["roll", "pitch", "yaw"][i]).value = (angle / DEG).toFixed(1); });
    saveTool();
  }
  function readJoints() {
    for (const name of state.joints) {
      const value = state.status?.joint_positions?.[name];
      if (Number.isFinite(value)) { $(name).value = (value / DEG).toFixed(1); $(`${name}-slider`).value = value / DEG; }
    }
    saveJoints();
  }
  function saveTool() { persist({ toolTarget: ["x", "y", "z", "roll", "pitch", "yaw"].map((id) => $(id).value) }); }
  function saveJoints() { persist({ jointTargets: Object.fromEntries(state.joints.map((name) => [name, $(name).value])) }); }
  for (const id of ["x", "y", "z", "roll", "pitch", "yaw"]) $(id).addEventListener("change", saveTool);
  $("named-pose").addEventListener("change", () => persist({ namedPose: $("named-pose").value }));
  $("part").addEventListener("change", () => persist({ part: $("part").value }));
  $("destination").addEventListener("change", () => persist({ destination: $("destination").value }));
  $("read-tool").addEventListener("click", readTool);
  $("read-joints").addEventListener("click", readJoints);
  $("plan-named").addEventListener("click", () => command(`plan ${$("named-pose").value}`));
  $("plan-tool").addEventListener("click", () => {
    saveTool();
    const values = ["x", "y", "z", "roll", "pitch", "yaw"].map((id) => $(id).value.trim() === "" ? NaN : Number($(id).value));
    if (!values.every(Number.isFinite)) { $("local-error").textContent = "Enter a finite value for every position and angle."; return; }
    publish(POSE, { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "world" }, pose: {
      position: { x: values[0], y: values[1], z: values[2] }, orientation: quaternion(...values.slice(3).map((value) => value * DEG)) } });
  });
  $("plan-joints").addEventListener("click", () => {
    saveJoints();
    const positions = state.joints.map((name) => $(name).value.trim() === "" ? NaN : Number($(name).value) * DEG);
    if (!positions.length || !positions.every((value, i) => Number.isFinite(value) && value >= state.status.joint_limits[state.joints[i]][0] && value <= state.status.joint_limits[state.joints[i]][1])) {
      $("local-error").textContent = "Enter a target within each joint's limits."; return;
    }
    publish(JOINTS, { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "" }, name: state.joints, position: positions, velocity: [], effort: [] });
  });
  $("pick-place").addEventListener("click", () => command(`pick_to ${$("part").value} ${$("destination").value}`));
  for (const action of ["execute", "clear", "open", "close"]) $(action).addEventListener("click", () => command(action));
  $("stop").addEventListener("click", () => { publish(EVENT, { data: "stop" }, true); publish(COMMAND, { data: "stop" }, true); });
  $("speed").addEventListener("input", () => { $("speed-label").textContent = `${$("speed").value}%`; });
  $("speed").addEventListener("change", () => command(`speed ${Number($("speed").value) / 100}`));

  function render() {
    const status = state.status || {};
    const live = fresh() && state.advertised;
    const idle = live && !status.busy && status.can_plan;
    options($("named-pose"), status.arm_poses || [], saved.namedPose || "ready");
    options($("part"), status.picks || [], saved.part);
    $("connection").textContent = live ? "LIVE WORKCELL" : "WAITING FOR WORKCELL";
    $("connection").style.color = live ? "#5dd7aa" : "#f0b95b";
    for (const id of ["plan-named", "plan-tool", "plan-joints", "speed", "named-pose", "part", "destination", "read-joints", "open", "close"]) $(id).disabled = !idle;
    $("plan-named").disabled ||= !$("named-pose").value;
    $("read-tool").disabled = !idle || !status.tool_pose;
    $("pick-place").disabled = !idle || !$("part").value || (status.held_parts || []).length > 0;
    $("open").disabled = !idle || !(status.poses || []).includes("open");
    $("close").disabled = !idle || !(status.poses || []).includes("close");
    $("execute").disabled = !live || !status.can_execute || status.busy;
    $("clear").disabled = !live || status.busy || !status.can_execute;
    $("stop").disabled = !state.advertised;
    $("mode").disabled = !live;
    if (JSON.stringify(state.joints) !== JSON.stringify(status.joints || [])) {
      state.joints = status.joints || [];
      $("joint-fields").replaceChildren(...state.joints.map((name, index) => {
        const row = document.createElement("div"); row.className = "joint";
        const label = document.createElement("label"); label.htmlFor = name; label.textContent = `J${index + 1}`; label.title = name;
        const slider = document.createElement("input"); slider.type = "range"; slider.id = `${name}-slider`; slider.setAttribute("aria-label", `${name} target slider`);
        const input = document.createElement("input"); input.type = "number"; input.id = name; input.setAttribute("aria-label", `${name} target degrees`);
        const limits = status.joint_limits[name];
        for (const field of [slider, input]) { field.min = (limits[0] / DEG).toFixed(2); field.max = (limits[1] / DEG).toFixed(2); field.step = "0.1"; }
        slider.addEventListener("input", () => { input.value = slider.value; }); input.addEventListener("input", () => { slider.value = input.value; });
        slider.addEventListener("change", saveJoints); input.addEventListener("change", saveJoints);
        const value = saved.jointTargets?.[name];
        if (value != null) input.value = slider.value = value;
        row.append(label, slider, input); return row;
      }));
      if (!saved.jointTargets) readJoints();
    }
    for (const name of state.joints) { $(name).disabled = !idle; $(`${name}-slider`).disabled = !idle; }
    if (root.activeElement !== $("speed")) { $("speed").value = Math.round((status.velocity_scale || 0.25) * 100); $("speed-label").textContent = `${$("speed").value}%`; }
    const phase = live ? status.state || "idle" : "disconnected";
    const titles = { idle: "Ready to plan", planning: "Planning collision-free path…", planned: "Preview ready — inspect, then execute", checking: "Checking the saved path…", executing: "Executing motion…", cancelling: "Stopping…", stopped: "Motion stopped", succeeded: "Motion complete", rejected: "Request rejected", failed: "Motion failed", invalidated: "Preview cleared — plan again", disconnected: "Waiting for live workcell" };
    $("status-name").textContent = titles[phase] || phase;
    $("status").dataset.tone = ["rejected", "failed", "invalidated", "disconnected"].includes(phase) ? "error" : status.busy ? "busy" : "ok";
    const target = status.pose || status.planned_pose;
    const detail = [target ? target.replaceAll("_", " ") : "", status.step ? `Step: ${status.step}` : "", status.reason || status.invalid_reason || "", status.error_code && status.error_code !== 1 ? status.error_name || `MoveIt code ${status.error_code}` : "", (status.held_parts || []).length ? `Holding: ${status.held_parts.join(", ")}` : ""].filter(Boolean);
    $("detail").textContent = live ? detail.join("\n") || "Choose a target and plan a motion." : "Connect Lichtblick to ws://localhost:8765 and start the workcell.";
    $("duration").textContent = status.can_execute ? `${status.plan_duration.toFixed(2)} s` : "—";
    $("points").textContent = status.can_execute ? String(status.plan_points) : "—";
    $("tool-readout").textContent = status.tool_pose ? `${status.tool_link} · world\nX ${status.tool_pose[0].toFixed(3)}  Y ${status.tool_pose[1].toFixed(3)}  Z ${status.tool_pose[2].toFixed(3)} m` : "Tool pose unavailable";
    const steps = ["open", "approach", "descend", "grasp", "attach", "lift", "carry", "lower", "detach", "release", "retreat"];
    $("steps").replaceChildren(...steps.map((name) => { const span = document.createElement("span"); span.className = `step${status.step === name ? " active" : ""}`; span.textContent = name; return span; }));
  }

  context.watch("currentFrame");
  context.subscribe([{ topic: STATUS }]);
  context.onRender = (frame, done) => {
    try {
      if (!state.advertised && context.advertise && context.publish) {
        context.advertise(COMMAND, "std_msgs/msg/String");
        context.advertise(POSE, "geometry_msgs/msg/PoseStamped");
        context.advertise(JOINTS, "sensor_msgs/msg/JointState");
        context.advertise(EVENT, "std_msgs/msg/String");
        state.advertised = true;
        $("local-error").textContent = "";
      }
      for (const event of frame.currentFrame || []) {
        if (event.topic !== STATUS) continue;
        const message = JSON.parse(event.message.data);
        state.status = message; state.received = clock();
        const key = JSON.stringify([message.state, message.pose, message.step, message.reason, message.velocity_scale]);
        if (key !== state.eventKey) {
          state.eventKey = key;
          state.events.unshift({ time: new Date().toLocaleTimeString(), text: [message.state, message.pose, message.step, message.reason].filter(Boolean).join(" · ") });
          state.events.length = Math.min(8, state.events.length);
          $("events").replaceChildren(...state.events.map((event) => { const item = document.createElement("li"); const time = document.createElement("time"); time.textContent = event.time; item.append(time, document.createTextNode(event.text)); return item; }));
        }
      }
      render();
    } catch (error) { $("local-error").textContent = error.message || String(error); }
    finally { done(); }
  };
  const timer = setInterval(render, 1000);
  render();
  return () => { clearInterval(timer); context.unsubscribeAll(); for (const topic of [COMMAND, POSE, JOINTS, EVENT]) context.unadvertise?.(topic); root.replaceChildren(); };
}

module.exports = { activate(context) { context.registerPanel({ name: "MoveIt controls", initPanel }); } };
