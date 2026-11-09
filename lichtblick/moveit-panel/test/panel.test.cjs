const { test } = require("node:test");
const assert = require("node:assert/strict");
const extension = require("../src/extension.js");

async function panel(t, patch = {}, initialState = {}) {
  const { Window } = await import("happy-dom");
  const window = new Window();
  const element = window.document.createElement("div");
  window.document.body.append(element);
  const messages = [];
  const advertised = [];
  const context = { panelElement: element, initialState, watch() {}, subscribe() {}, unsubscribeAll() {},
    saveState(value) { context.savedState = value; }, advertise(...args) { advertised.push(args); }, unadvertise() {}, publish(...args) { messages.push(args); } };
  let registration;
  extension.activate({ registerPanel(value) { registration = value; } });
  const cleanup = registration.initPanel(context);
  t.after(() => { cleanup(); window.happyDOM.abort(); });
  const status = { state: "idle", busy: false, can_plan: true, can_execute: false, velocity_scale: 0.25,
    arm_poses: ["home", "ready", "sweep"], picks: ["pick_cube", "pick_cylinder"], poses: ["home", "ready", "sweep", "open", "close"],
    joints: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    joint_limits: Object.fromEntries([1,2,3,4,5,6].map((n) => [`joint${n}`, [-Math.PI, Math.PI]])),
    joint_positions: Object.fromEntries([1,2,3,4,5,6].map((n) => [`joint${n}`, 0])),
    held_parts: [], tool_link: "link_tcp", tool_pose: [0.3, 0.1, 1.2, 1, 0, 0, 0], ...patch };
  let completed = 0;
  const update = (value) => context.onRender({ currentFrame: [{ topic: "/motion/status", message: { data: JSON.stringify(value) } }] }, () => { completed++; });
  const $ = (id) => element.shadowRoot.getElementById(id);
  const change = (id, value) => { $(id).value = value; $(id).dispatchEvent(new window.Event("change")); };
  update(status);
  assert.equal(completed, 1);
  return { $, update, status, messages, advertised, change, context };
}

test("named pose plans before enabling execution; busy controls reject clicks", async (t) => {
  const p = await panel(t);
  assert.equal(p.$("named-pose").value, "ready");
  assert.equal(p.$("execute").disabled, true);
  p.$("plan-named").click();
  assert.deepEqual(p.messages.at(-1), ["/motion/command", { data: "plan ready" }]);
  p.update({ ...p.status, state: "planned", can_execute: true, plan_duration: 2.75, plan_points: 42 });
  assert.equal(p.$("execute").disabled, false);
  assert.equal(p.$("duration").textContent, "2.75 s");
  p.$("execute").click();
  assert.deepEqual(p.messages.at(-1), ["/motion/command", { data: "execute" }]);
  p.update({ ...p.status, state: "executing", busy: true });
  const count = p.messages.length;
  p.$("plan-named").click();
  assert.equal(p.messages.length, count);
  p.$("stop").click();
  assert.deepEqual(p.messages.at(-1), ["/motion/command", { data: "stop" }]);
});

test("Cartesian mode copies current pose and publishes normalized orientation", async (t) => {
  const p = await panel(t);
  p.change("mode", "cartesian");
  assert.equal(p.$("named").hidden, true);
  assert.equal(p.$("cartesian").hidden, false);
  p.$("read-tool").click();
  p.$("x").value = "0.4";
  p.$("plan-tool").click();
  const [topic, message] = p.messages.at(-1);
  assert.equal(topic, "/motion/goal_pose");
  assert.equal(message.header.frame_id, "world");
  assert.deepEqual(message.pose.position, { x: 0.4, y: 0.1, z: 1.2 });
  assert.ok(Math.abs(message.pose.orientation.x - 1) < 1e-12);
  assert.ok(Math.abs(message.pose.orientation.w) < 1e-12);
  const count = p.messages.length;
  p.$("x").value = "";
  p.$("plan-tool").click();
  assert.equal(p.messages.length, count);
  assert.match(p.$("local-error").textContent, /finite/);
});

test("joint targets convert degrees to radians and enforce joint limits", async (t) => {
  const p = await panel(t);
  p.change("mode", "joints");
  p.$("joint1").value = "90";
  p.$("plan-joints").click();
  const [topic, message] = p.messages.at(-1);
  assert.equal(topic, "/motion/goal_joints");
  assert.equal(message.name.length, 6);
  assert.equal(message.position[0], Math.PI / 2);
  p.$("joint1").value = "999";
  const count = p.messages.length;
  p.$("plan-joints").click();
  assert.equal(p.messages.length, count);
  assert.match(p.$("local-error").textContent, /limits/);
});

test("pick controls select a part and explicit destination and show held parts", async (t) => {
  const p = await panel(t);
  p.change("mode", "pick");
  p.change("part", "pick_cylinder");
  p.change("destination", "home");
  p.$("pick-place").click();
  assert.deepEqual(p.messages.at(-1), ["/motion/command", { data: "pick_to pick_cylinder home" }]);
  p.update({ ...p.status, held_parts: ["pick_cube"] });
  assert.equal(p.$("pick-place").disabled, true);
  assert.match(p.$("detail").textContent, /Holding: pick_cube/);
  p.$("open").click();
  assert.deepEqual(p.messages.at(-1), ["/motion/command", { data: "open" }]);
});

test("speed publishes a bounded scale and a stale status disables motion", async (t) => {
  const p = await panel(t);
  p.change("speed", "10");
  assert.deepEqual(p.messages.at(-1), ["/motion/command", { data: "speed 0.1" }]);
  t.mock.method(Date, "now", () => Date.parse("2100-01-01T00:00:00Z"));
  p.context.onRender({ currentFrame: [] }, () => {});
  assert.equal(p.$("execute").disabled, true);
  assert.equal(p.$("plan-named").disabled, true);
  assert.match(p.$("connection").textContent, /WAITING/);
});

test("edited targets and dropdown choices survive closing the tab", async (t) => {
  const p = await panel(t);
  p.change("mode", "cartesian");
  p.change("x", "0.456");
  p.change("part", "pick_cylinder");
  p.change("destination", "drop");
  const reopened = await panel(t, {}, p.context.savedState);
  assert.equal(reopened.$("mode").value, "cartesian");
  assert.equal(reopened.$("x").value, "0.456");
  assert.equal(reopened.$("part").value, "pick_cylinder");
  assert.equal(reopened.$("destination").value, "drop");
});
