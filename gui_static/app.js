// policy_bridge CaCoFlow dashboard -- vanilla JS. Server-provided strings (task names, errors, logs, history)
// are only ever rendered with textContent, never innerHTML.
const $ = (id) => document.getElementById(id);
let state = null;
let actionError = "";
let lastTasks = "";
let lastMethods = "";
let cfgZoom = 1.2;
const METHOD_ROWS = 3;

// ------------------------------------------------------------------------------ i18n / tabs
document.querySelectorAll(".lang-toggle button").forEach((b) => b.addEventListener("click", () => setLang(b.dataset.lang)));
function onLangChange() { if (state) render(); loadHistory(); }

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
const mGrid = $("methods-grid");
for (let i = 0; i < METHOD_ROWS; i++) {
  const row = document.createElement("div"); row.className = "row";
  for (const [id, label, type, w] of [[`m-name-${i}`, "name", "text", "1 1 160px"], [`m-port-${i}`, "remote port", "number", "0 1 120px"],
    [`m-url-${i}`, "direct URL (url mode)", "text", "2 1 240px"]]) {
    const l = document.createElement("label"); l.style.flex = w;
    const s = document.createElement("span"); s.textContent = label;
    const inp = document.createElement("input"); inp.type = type; inp.id = id;
    l.append(s, inp); row.append(l);
  }
  mGrid.append(row);
}

function fillConfig(c) {
  $("robot-url").value = c.robot_url;
  $("policy-mode").value = c.policy_mode;
  $("ssh-host").value = c.ssh_host; $("ssh-port").value = c.ssh_port; $("ssh-user").value = c.ssh_user;
  $("ssh-key").value = c.ssh_key; $("policy-url").value = c.policy_url;
  $("agent-zoom").value = c.agent_zoom; $("timeout").value = c.timeout; $("max-age").value = c.max_observation_age;
  $("cfg-auto-record").checked = c.auto_record;
  for (let i = 0; i < METHOD_ROWS; i++) {
    const m = c.methods[i];
    $(`m-name-${i}`).value = m ? m.name : ""; $(`m-port-${i}`).value = m ? m.remote_policy_port : "";
    $(`m-url-${i}`).value = m ? m.policy_url : "";
  }
  c.joint_impedance.forEach((v, i) => { $(`imp-${i}`).value = v; });
  $("v-max").value = c.v_max; $("a-max").value = c.a_max; $("j-max").value = c.j_max; $("filter-wn").value = c.filter_wn;
  $("auto-start").checked = c.auto_start_server;
  cfgZoom = c.agent_zoom;
  syncModeFields();
}

function readConfig() {
  const n = (id) => Number($(id).value);
  const methods = [];
  for (let i = 0; i < METHOD_ROWS; i++) {
    const name = $(`m-name-${i}`).value.trim();
    if (name) methods.push({ name, remote_policy_port: n(`m-port-${i}`) || 8000, policy_url: $(`m-url-${i}`).value.trim() });
  }
  return {
    robot_url: $("robot-url").value.trim(), policy_mode: $("policy-mode").value,
    ssh_host: $("ssh-host").value.trim(), ssh_port: n("ssh-port"), ssh_user: $("ssh-user").value.trim(),
    ssh_key: $("ssh-key").value.trim(), policy_url: $("policy-url").value.trim(), methods,
    agent_zoom: n("agent-zoom"), timeout: n("timeout"), max_observation_age: n("max-age"),
    auto_record: $("cfg-auto-record").checked,
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
const effectiveInstruction = () => $("instruction-custom").value.trim() || $("instruction").value;

$("btn-prepare").addEventListener("click", () => act(() => api("POST", "/api/robot/prepare")));
$("btn-robot-disc").addEventListener("click", () => act(() => api("POST", "/api/robot/disconnect")));
$("btn-server-stop").addEventListener("click", () => act(() => api("POST", "/api/server/stop")));
$("btn-policy-connect").addEventListener("click", () => act(async () => {
  await api("POST", "/api/policy/connect", { token: $("token").value, ssh_password: $("ssh-password").value, method: $("method").value || null });
}));
$("btn-policy-disc").addEventListener("click", () => act(() => api("POST", "/api/policy/disconnect")));
$("btn-rec-start").addEventListener("click", () => act(() => api("POST", "/api/record/start", { label: "manual" })));
$("btn-rec-stop").addEventListener("click", () => act(() => api("POST", "/api/record/stop")));
$("btn-stop").addEventListener("click", () => act(() => api("POST", "/api/trial/stop")));
$("btn-start").addEventListener("click", () => act(async () => {
  if (!$("gripper-one").value) throw new Error(t("msg.need_gripper"));
  const opt = (id) => $(id).value.trim() || null;
  await api("POST", "/api/trial/start", {
    instruction: effectiveInstruction(), gripper_one_means: $("gripper-one").value,
    trial_id: opt("trial-id"), placement: opt("placement"), appearance: $("appearance").value,
    max_steps: Number($("max-steps").value), agent_zoom: cfgZoom, record: $("auto-record").checked,
    estop_ready: $("estop-ready").checked,
  });
  $("estop-ready").checked = false;          // re-confirm for every trial
  $("trial-id").value = "";                  // next trial gets a fresh automatic id
}));

function sendOutcome(success) {
  return act(async () => {
    const r = await api("POST", "/api/trial/outcome", {
      success, wrong_object: $("out-wrong").value === "" ? null : $("out-wrong").value === "true", notes: $("out-notes").value,
    });
    $("out-wrong").value = ""; $("out-notes").value = "";
    if (r.server_error) { actionError = t("outcome.server_error", { err: r.server_error }); }
    else showBanner(t("outcome.saved", { dir: r.trial_dir || "-" }), "info");
    loadHistory();
  });
}
$("btn-success").addEventListener("click", () => sendOutcome(true));
$("btn-failure").addEventListener("click", () => sendOutcome(false));
$("btn-undecided").addEventListener("click", () => sendOutcome(null));

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

function fillSelect(sel, values) {
  const prev = sel.value;
  sel.replaceChildren();
  for (const v of values) { const o = document.createElement("option"); o.value = v; o.textContent = v; sel.append(o); }
  if (values.includes(prev)) sel.value = prev;
}

function render() {
  const s = state; if (!s) return;
  const badge = badgeOf(s);
  const bd = $("state-badge"); bd.className = "badge " + badge; bd.textContent = t("badge." + badge);

  let msg = "";
  if (actionError) msg = actionError;
  else if (s.trial.state === "error") msg = t("err.trial") + s.trial.error;
  else if (s.bringup.step === "error") msg = t("err.bringup") + s.bringup.message;
  else if (s.robot.error) msg = t("err.robot") + s.robot.error;
  else if (s.recording.error) msg = t("err.record") + s.recording.error;
  else if (s.server.message) msg = s.server.message;
  if (msg || !$("banner").classList.contains("info")) showBanner(msg);

  let line = t("srv." + s.server.state);
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

  // methods + instructions
  const mkey = (s.methods || []).join("\n");
  if (mkey !== lastMethods) { lastMethods = mkey; fillSelect($("method"), s.methods || []); }
  $("policy-method").textContent = s.policy.connected ? t("policy.connected_to", { m: s.policy.method }) : "";
  renderInfo(s.policy.info);
  const tasks = (s.policy.info && s.policy.info.tasks) || [];
  if (tasks.join("\n") !== lastTasks) { lastTasks = tasks.join("\n"); fillSelect($("instruction"), tasks); }
  const custom = $("instruction-custom").value.trim();
  $("custom-warn").classList.toggle("hidden", !(custom && !tasks.includes(custom)));

  // recording
  const rec = s.recording;
  $("rec-status").textContent = rec.active ? t("rec.active", { s: f(rec.seconds, 0), n: rec.frames })
    : (rec.path ? t("rec.last", { p: rec.path, n: rec.frames }) : "");

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

  // buttons
  const busy = !!s.busy;
  const canStart = s.policy.connected && r.connected && s.trial.state === "idle" && !busy
    && $("gripper-one").value && $("estop-ready").checked && effectiveInstruction();
  $("btn-start").disabled = !canStart;
  $("btn-stop").disabled = !running;
  $("btn-prepare").disabled = busy || running || r.connected || s.server.state === "starting";
  $("btn-robot-disc").disabled = busy || running || !r.connected;
  $("btn-server-stop").disabled = busy || running || r.connected || s.server.state === "stopped";
  $("btn-policy-connect").disabled = busy || running;
  $("btn-policy-disc").disabled = busy || running || !s.policy.connected;
  $("btn-rec-start").disabled = !r.connected || rec.active;
  $("btn-rec-stop").disabled = !rec.active || running;
  $("panel-outcome").classList.toggle("hidden", s.trial.state !== "awaiting_outcome");
  $("data-dir").textContent = t("hist.dir", { d: s.data_dir });
  for (const id of ["instruction", "instruction-custom", "gripper-one", "trial-id", "placement", "appearance", "max-steps",
    "estop-ready", "auto-record"]) $(id).disabled = running;
}
["gripper-one", "estop-ready", "instruction"].forEach((id) => $(id).addEventListener("change", render));
$("instruction-custom").addEventListener("input", render);

async function refresh() {
  try { state = await api("GET", "/api/state"); } catch (e) { actionError = e.message; }
  render();
}

// ---------------------------------------------------------------------------------- history
async function loadHistory() {
  let h;
  try { h = await api("GET", "/api/history"); } catch (e) { return; }
  const parts = Object.entries(h.summary).map(([m, s]) => {
    const done = s.success + s.failure;
    const rate = done ? ` (${Math.round((100 * s.success) / done)}%)` : "";
    return t("hist.summary", { m, s: s.success, f: s.failure, u: s.undecided, e: s.error, rate });
  });
  $("hist-summary").textContent = parts.join("   |   ") || t("hist.none");
  const tb = $("hist-body"); tb.replaceChildren();
  for (const r of h.rows) {
    const tr = document.createElement("tr");
    const res = r.status === "error" ? t("hist.error") : r.success === "True" ? t("hist.ok") : r.success === "False" ? t("hist.fail") : t("hist.und");
    const vid = r.video ? r.video.split("/").pop() : "";
    for (const v of [r.time, r.method, r.trial_id, r.instruction, res, vid]) {
      const td = document.createElement("td"); td.textContent = v; tr.append(td);
    }
    tb.append(tr);
  }
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
api("GET", "/api/config").then((c) => { fillConfig(c); $("auto-record").checked = c.auto_record; })
  .catch((e) => { actionError = e.message; });
refresh();
loadHistory();
setInterval(refresh, 500);
setInterval(tickCams, 300);
setInterval(loadHistory, 5000);
