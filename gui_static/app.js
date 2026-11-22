// policy_bridge CaCoFlow dashboard -- vanilla JS. Server-provided strings (task names, errors, logs) are only
// ever rendered with textContent, never innerHTML.
const $ = (id) => document.getElementById(id);
let state = null;
let actionError = "";
let lastTasks = "";

// ------------------------------------------------------------------------------ i18n / tabs
document.querySelectorAll(".lang-toggle button").forEach((b) => b.addEventListener("click", () => setLang(b.dataset.lang)));
function onLangChange() { if (state) render(); }

function setTab(name) {
  document.querySelectorAll(".tab-page").forEach((el) => el.classList.toggle("hidden", el.dataset.tab !== name));
  document.querySelectorAll(".tab-btn").forEach((b) => b.classList.toggle("active", b.dataset.tab === name));
  try { localStorage.setItem("policy-bridge-tab", name); } catch (e) { /* ignore */ }
}
document.querySelectorAll(".tab-btn").forEach((b) => b.addEventListener("click", () => setTab(b.dataset.tab)));
let initialTab = "deploy";
try { initialTab = localStorage.getItem("policy-bridge-tab") || "deploy"; } catch (e) { /* ignore */ }
setTab(initialTab);

// ------------------------------------------------------------------------------------ API
async function api(method, path, body) {
  const res = await fetch(path, {
    method, headers: body === undefined ? {} : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  let data = null;
  try { data = await res.json(); } catch (e) { /* not JSON */ }
  if (!res.ok) {
    let d = data && data.detail;
    if (Array.isArray(d)) d = d.map((x) => `${(x.loc || []).slice(1).join(".")}: ${x.msg}`).join("; ");
    throw new Error(d || `HTTP ${res.status}`);
  }
  return data;
}

async function act(fn) {
  actionError = "";
  showBanner("");
  try { await fn(); } catch (e) { actionError = e.message; }
  await refresh();
}

// ------------------------------------------------------------------------------- settings
const DEFAULT_IMPEDANCE = [3000, 3000, 3000, 2500, 2500, 2000, 2000];
const impGrid = $("imp-grid");
for (let i = 0; i < 7; i++) {
  const l = document.createElement("label");
  const s = document.createElement("span"); s.textContent = `J${i + 1}`;
  const inp = document.createElement("input"); inp.type = "number"; inp.step = "50"; inp.id = `imp-${i}`;
  l.append(s, inp); impGrid.append(l);
}

function fillConfig(c) {
  $("robot-url").value = c.robot_url;
  $("policy-mode").value = c.policy_mode;
  $("ssh-host").value = c.ssh_host; $("ssh-port").value = c.ssh_port; $("ssh-user").value = c.ssh_user;
  $("ssh-key").value = c.ssh_key; $("remote-port").value = c.remote_policy_port; $("policy-url").value = c.policy_url;
  $("agent-zoom").value = c.agent_zoom; $("timeout").value = c.timeout; $("max-age").value = c.max_observation_age;
  c.joint_impedance.forEach((v, i) => { $(`imp-${i}`).value = v; });
  $("v-max").value = c.v_max; $("a-max").value = c.a_max; $("j-max").value = c.j_max; $("filter-wn").value = c.filter_wn;
  $("auto-start").checked = c.auto_start_server;
  syncModeFields();
}

function readConfig() {
  const n = (id) => Number($(id).value);
  return {
    robot_url: $("robot-url").value.trim(), policy_mode: $("policy-mode").value,
    ssh_host: $("ssh-host").value.trim(), ssh_port: n("ssh-port"), ssh_user: $("ssh-user").value.trim(),
    ssh_key: $("ssh-key").value.trim(), remote_policy_port: n("remote-port"), policy_url: $("policy-url").value.trim(),
    agent_zoom: n("agent-zoom"), timeout: n("timeout"), max_observation_age: n("max-age"),
    joint_impedance: [0, 1, 2, 3, 4, 5, 6].map((i) => n(`imp-${i}`)),
    v_max: n("v-max"), a_max: n("a-max"), j_max: n("j-max"), filter_wn: n("filter-wn"),
    auto_start_server: $("auto-start").checked,
  };
}

function syncModeFields() {
  const ssh = $("policy-mode").value === "ssh";
  $("ssh-fields").classList.toggle("hidden", !ssh);
  $("url-fields").classList.toggle("hidden", ssh);
}
$("policy-mode").addEventListener("change", syncModeFields);
$("btn-defaults").addEventListener("click", () => {
  DEFAULT_IMPEDANCE.forEach((v, i) => { $(`imp-${i}`).value = v; });
  $("v-max").value = 0.8; $("a-max").value = 2.5; $("j-max").value = 1500; $("filter-wn").value = 8;
});
$("btn-save").addEventListener("click", () => act(async () => {
  const saved = await api("POST", "/api/config", readConfig());
  fillConfig(saved);
  $("save-msg").textContent = t("conn.saved");
  setTimeout(() => { $("save-msg").textContent = ""; }, 2500);
}));

// -------------------------------------------------------------------------------- actions
$("btn-prepare").addEventListener("click", () => act(() => api("POST", "/api/robot/prepare")));
$("btn-robot-disc").addEventListener("click", () => act(() => api("POST", "/api/robot/disconnect")));
$("btn-server-stop").addEventListener("click", () => act(() => api("POST", "/api/server/stop")));
$("btn-policy-connect").addEventListener("click", () => act(async () => {
  await api("POST", "/api/policy/connect", { token: $("token").value, ssh_password: $("ssh-password").value });
}));
$("btn-policy-disc").addEventListener("click", () => act(() => api("POST", "/api/policy/disconnect")));
$("btn-stop").addEventListener("click", () => act(() => api("POST", "/api/trial/stop")));
$("btn-start").addEventListener("click", () => act(async () => {
  if (!$("gripper-one").value) throw new Error(t("msg.need_gripper"));
  const opt = (id) => $(id).value.trim() || null;
  await api("POST", "/api/trial/start", {
    instruction: $("instruction").value, gripper_one_means: $("gripper-one").value,
    trial_id: opt("trial-id"), placement: opt("placement"), appearance: $("appearance").value,
    max_steps: Number($("max-steps").value), agent_zoom: Number($("agent-zoom").value) || 1.0,
    estop_ready: $("estop-ready").checked,
  });
  $("estop-ready").checked = false;          // re-confirm for every trial
}));
$("btn-outcome").addEventListener("click", () => act(async () => {
  const tri = (v) => (v === "" ? null : v === "true");
  const r = await api("POST", "/api/trial/outcome", {
    success: tri($("out-success").value), wrong_object: tri($("out-wrong").value), notes: $("out-notes").value,
  });
  $("out-success").value = ""; $("out-wrong").value = ""; $("out-notes").value = "";
  actionError = ""; showBanner(t("outcome.saved", { dir: r.trial_dir || "-" }), "info");
}));

// ---------------------------------------------------------------------------------- render
function showBanner(msg, kind) {
  const b = $("banner");
  b.textContent = msg || "";
  b.classList.toggle("hidden", !msg);
  b.classList.toggle("info", kind === "info");
}

function stats(el, items) {
  el.replaceChildren();
  for (const it of items) {
    const d = document.createElement("div"); d.className = "stat" + (it.cls ? " " + it.cls : "");
    const l = document.createElement("div"); l.className = "stat-label"; l.textContent = it.label;
    const v = document.createElement("div"); v.className = "stat-value"; v.textContent = it.value;
    d.append(l, v); el.append(d);
  }
}
const f = (v, d = 0) => (v === null || v === undefined ? "-" : Number(v).toFixed(d));

function badgeOf(s) {
  const ts = s.trial.state;
  if (ts === "running") return "running";
  if (ts === "awaiting_outcome") return "awaiting_outcome";
  if (ts === "error" || s.bringup.step === "error") return "error";
  if (["starting", "server", "robot"].includes(s.bringup.step)) return "preparing";
  if (s.robot.connected && s.policy.connected) return "ready";
  if (s.robot.connected || s.policy.connected) return "connected";
  return "idle";
}

function renderInfo(info) {
  const tb = $("info-body"); tb.replaceChildren();
  $("info-table").classList.toggle("hidden", !info);
  if (!info) return;
  const rows = [["info.model_id", info.model_id], ["info.mock", String(info.mock)],
    ["info.action_shape", JSON.stringify(info.action_shape)], ["info.sampling_steps", info.sampling_steps],
    ["info.chunk_size", info.chunk_size], ["info.tasks", (info.tasks || []).join(" / ")]];
  for (const [k, v] of rows) {
    const tr = document.createElement("tr");
    const a = document.createElement("td"); a.className = "k"; a.textContent = t(k);
    const b = document.createElement("td"); b.textContent = v === undefined ? "-" : String(v);
    tr.append(a, b); tb.append(tr);
  }
}

function renderInstructions(info) {
  const tasks = (info && info.tasks) || [];
  const key = tasks.join("\n");
  if (key === lastTasks) return;
  lastTasks = key;
  const sel = $("instruction"); const prev = sel.value;
  sel.replaceChildren();
  for (const task of tasks) { const o = document.createElement("option"); o.value = task; o.textContent = task; sel.append(o); }
  if (tasks.includes(prev)) sel.value = prev;
}

function render() {
  const s = state; if (!s) return;
  const badge = badgeOf(s);
  const bd = $("state-badge"); bd.className = "badge " + badge; bd.textContent = t("badge." + badge);

  // banners: most important first
  let msg = "";
  if (actionError) msg = actionError;
  else if (s.trial.state === "error") msg = t("err.trial") + s.trial.error;
  else if (s.bringup.step === "error") msg = t("err.bringup") + s.bringup.message;
  else if (s.robot.error) msg = t("err.robot") + s.robot.error;
  else if (s.server.message) msg = s.server.message;
  if (msg || !$("banner").classList.contains("info")) showBanner(msg);

  // server / robot
  const srvKey = "srv." + s.server.state;
  let line = t(srvKey);
  if (s.bringup.step === "server") line += "  · " + t("bring.server");
  if (s.bringup.step === "robot") line += "  · " + t("bring.robot");
  if (s.server.restart_needed) line += t("srv.restart");
  $("server-status").textContent = line;
  $("server-log").textContent = (s.server.log || []).join("\n");

  const r = s.robot;
  const items = [{ label: t("stat.robot"), value: r.connected ? t("stat.connected") : t("stat.disconnected"),
    cls: r.connected ? "stat-ok" : "" }];
  if (r.connected) {
    items.push({ label: t("stat.gripper"), value: f(r.gripper_closed, 2) },
      { label: t("stat.state_age"), value: f(r.state_age_ms) + " ms", cls: r.state_age_ms > 100 ? "stat-warn" : "" },
      { label: t("stat.image_age"), value: f(r.image_age_ms) + " ms", cls: r.image_age_ms > 300 ? "stat-warn" : "" },
      { label: t("stat.joints"), value: (r.q || []).map((v) => v.toFixed(2)).join(" ") });
  }
  stats($("telemetry-grid"), items);

  renderInfo(s.policy.info);
  renderInstructions(s.policy.info);

  // trial progress
  const p = s.trial.progress || {}; const running = s.trial.state === "running";
  if (p.max_steps) {
    const res = s.trial.result;
    stats($("progress-grid"), [
      { label: t("prog.step"), value: `${p.step ?? 0} / ${p.max_steps}` },
      { label: t("prog.chunks"), value: String(p.chunks ?? 0) },
      { label: t("prog.rt"), value: `${f(p.last_rt_ms)} / ${f(p.rt_median_ms)} / ${f(p.rt_max_ms)} ms` },
      { label: t("prog.elapsed"), value: f(p.elapsed_s, 1) },
      ...(res ? [{ label: t("prog.clipped"), value: String(res.clipped_steps) }] : []),
      ...(p.stopped ? [{ label: t("prog.stopped"), value: "✔", cls: "stat-warn" }] : []),
    ]);
  } else { $("progress-grid").replaceChildren(); }

  // button states
  const busy = !!s.busy;
  const canStart = s.policy.connected && r.connected && s.trial.state === "idle" && !busy
    && $("gripper-one").value && $("estop-ready").checked && $("instruction").value;
  $("btn-start").disabled = !canStart;
  $("btn-stop").disabled = !running;
  $("btn-prepare").disabled = busy || running || r.connected || s.server.state === "starting";
  $("btn-robot-disc").disabled = busy || running || !r.connected;
  $("btn-server-stop").disabled = busy || running || r.connected || s.server.state === "stopped";
  $("btn-policy-connect").disabled = busy || running;
  $("btn-policy-disc").disabled = busy || running || !s.policy.connected;
  $("panel-outcome").classList.toggle("hidden", s.trial.state !== "awaiting_outcome");
  for (const id of ["instruction", "gripper-one", "trial-id", "placement", "appearance", "max-steps", "estop-ready"]) $(id).disabled = running;
}
["gripper-one", "estop-ready", "instruction"].forEach((id) => $(id).addEventListener("change", render));

async function refresh() {
  try { state = await api("GET", "/api/state"); } catch (e) { actionError = e.message; }
  render();
}

// ----------------------------------------------------------------------------------- polling
function tickCams() {
  if (!state || !state.robot.connected || document.hidden) return;
  if (document.querySelector('.tab-page[data-tab="deploy"]').classList.contains("hidden")) return;
  for (const role of ["agent", "wrist"]) {
    const img = $(`cam-${role}`); const probe = new Image();
    probe.onload = () => { img.src = probe.src; };
    probe.src = `/api/camera/${role}.jpg?t=${Date.now()}`;
  }
}

setLang(getLang());
api("GET", "/api/config").then(fillConfig).catch((e) => { actionError = e.message; });
refresh();
setInterval(refresh, 500);
setInterval(tickCams, 300);
