// Minimal i18n (same approach as franka-deploy): [data-i18n]/[data-i18n-ph] for static markup,
// t(key, vars) for JS-generated text. Default Korean, EN toggle.
const I18N = {
  "tab.settings": { ko: "설정", en: "Settings" },
  "tab.deploy": { ko: "배포", en: "Deploy" },

  "robot.title": { ko: "로봇 (Robot Server)", en: "Robot (Robot Server)" },
  "robot.hint": {
    ko: "서버가 꺼져 있으면 RT 튜닝, 카메라 리셋, 컨트롤러 설정(설정 탭)을 거쳐 자동으로 시작한 뒤 연결합니다. <b>연결하는 순간 1 kHz 컨트롤러가 켜지고 팔이 임피던스 제어로 고정됩니다.</b>",
    en: "If the server is off, it is started automatically (RT tuning, camera reset, controller settings from the Settings tab) and then connected. <b>Connecting starts the 1 kHz controller and holds the arm under impedance control.</b>",
  },
  "robot.prepare": { ko: "로봇 준비 (서버 시작 + 연결)", en: "Prepare robot (start server + connect)" },
  "robot.disconnect": { ko: "로봇 연결 해제", en: "Disconnect robot" },
  "robot.server_stop": { ko: "서버 중지", en: "Stop server" },
  "robot.log": { ko: "서버 로그", en: "Server log" },
  "srv.stopped": { ko: "서버: 꺼짐", en: "Server: stopped" },
  "srv.starting": { ko: "서버: 시작 중 (RT 튜닝·카메라 리셋, 1–2분 걸릴 수 있음)", en: "Server: starting (RT tuning, camera reset; may take 1–2 min)" },
  "srv.running": { ko: "서버: 실행 중", en: "Server: running" },
  "srv.restart": { ko: " — 컨트롤러 설정이 바뀌었습니다. 서버를 중지하고 다시 준비하세요", en: " — controller settings changed; stop the server and prepare again" },
  "bring.server": { ko: "Robot Server 시작 중…", en: "Starting Robot Server…" },
  "bring.robot": { ko: "로봇에 연결 중…", en: "Connecting to robot…" },
  "stat.robot": { ko: "로봇", en: "Robot" },
  "stat.connected": { ko: "연결됨", en: "connected" },
  "stat.disconnected": { ko: "연결 안 됨", en: "disconnected" },
  "stat.gripper": { ko: "그리퍼(서버 기준 1=닫힘)", en: "Gripper (server: 1=closed)" },
  "stat.state_age": { ko: "상태 나이", en: "State age" },
  "stat.image_age": { ko: "이미지 나이", en: "Image age" },
  "stat.joints": { ko: "관절 q (rad)", en: "Joints q (rad)" },

  "policy.title": { ko: "CaCoFlow 서버", en: "CaCoFlow server" },
  "policy.token": { ko: "API 토큰 (메모리에만 보관, 저장 안 됨)", en: "API token (kept in memory only, never saved)" },
  "policy.token.ph": { ko: "비우면 환경변수 CACOFLOW_API_TOKEN 사용", en: "empty = use env CACOFLOW_API_TOKEN" },
  "policy.sshpw": { ko: "SSH 비밀번호 (키 로그인이면 비움)", en: "SSH password (leave empty for key login)" },
  "policy.connect": { ko: "서버 연결 / 확인", en: "Connect / check server" },
  "policy.disconnect": { ko: "연결 해제", en: "Disconnect" },

  "trial.title": { ko: "시행", en: "Trial" },
  "trial.instruction": { ko: "작업 (서버 /info 목록)", en: "Task (from the server's /info)" },
  "trial.gripper": { ko: "그리퍼 값 1의 의미 (필수)", en: "Meaning of gripper value 1 (required)" },
  "trial.choose": { ko: "-- 선택 --", en: "-- choose --" },
  "trial.closed": { ko: "닫힘 (closed)", en: "closed" },
  "trial.open": { ko: "열림 (open)", en: "open" },
  "trial.trial_id": { ko: "trial id (선택)", en: "trial id (optional)" },
  "trial.placement": { ko: "placement (선택)", en: "placement (optional)" },
  "trial.appearance": { ko: "appearance", en: "appearance" },
  "trial.max_steps": { ko: "최대 step (30 Hz)", en: "Max steps (30 Hz)" },
  "trial.hint": {
    ko: "첫 시행은 낮은 위치에서 짧게(60 step ≈ 2 s) 시험하세요. 학습 때의 컨트롤러와 reference filter가 다르면 로봇이 학습 때와 다른 속도로 움직입니다 (CACOFLOW.md).",
    en: "Run the first trial short (60 steps ≈ 2 s) from a low position. If the training-time controller/reference filter differs, the robot moves at a different speed than in training (CACOFLOW.md).",
  },
  "trial.estop": { ko: "비상 정지 버튼이 손 닿는 곳에 있음", en: "E-stop is within reach" },
  "trial.start": { ko: "시행 시작", en: "Start trial" },
  "trial.stop": { ko: "정지 (현재 위치 유지)", en: "Stop (hold position)" },
  "trial.stop_hint": {
    ko: "정지는 추론 응답을 기다리는 중이면 그 응답이 끝난 뒤 반영됩니다(그동안 로봇은 마지막 목표를 유지). 실제 안전장치는 물리 비상 정지입니다.",
    en: "If a prediction is in flight, Stop takes effect once it returns (the robot holds its last target meanwhile). The physical e-stop is the real safety device.",
  },
  "prog.step": { ko: "step", en: "Step" },
  "prog.chunks": { ko: "chunk", en: "Chunks" },
  "prog.rt": { ko: "왕복 지연 (최근/중앙값/최대)", en: "Round trip (last/median/max)" },
  "prog.elapsed": { ko: "경과 (s)", en: "Elapsed (s)" },
  "prog.clipped": { ko: "clip된 step", en: "Clipped steps" },
  "prog.stopped": { ko: "정지됨", en: "Stopped" },
  "info.model_id": { ko: "model_id", en: "model_id" },
  "info.mock": { ko: "mock", en: "mock" },
  "info.action_shape": { ko: "action_shape", en: "action_shape" },
  "info.sampling_steps": { ko: "sampling_steps", en: "sampling_steps" },
  "info.chunk_size": { ko: "chunk_size", en: "chunk_size" },
  "info.tasks": { ko: "작업 목록", en: "Tasks" },

  "outcome.title": { ko: "결과 입력", en: "Outcome" },
  "outcome.success": { ko: "성공 여부", en: "Success" },
  "outcome.undecided": { ko: "미결정", en: "Undecided" },
  "outcome.yes": { ko: "성공", en: "Success" },
  "outcome.no": { ko: "실패", en: "Failure" },
  "outcome.wrong": { ko: "다른 물체 잡음", en: "Wrong object" },
  "outcome.na": { ko: "해당 없음", en: "n/a" },
  "outcome.yes2": { ko: "예", en: "Yes" },
  "outcome.no2": { ko: "아니오", en: "No" },
  "outcome.notes": { ko: "메모", en: "Notes" },
  "outcome.send": { ko: "서버에 전송 (/end_trial)", en: "Send to server (/end_trial)" },
  "outcome.saved": { ko: "기록됨: {dir}", en: "Recorded: {dir}" },

  "ctl.title": { ko: "컨트롤러 설정 (Robot Server)", en: "Controller settings (Robot Server)" },
  "ctl.hint": {
    ko: "서버를 (재)시작할 때 적용됩니다. 기본값은 데이터 수집기(JOINT_STIFFNESS)와 서버 기본값이 같은 값입니다. 학습 데이터의 컨트롤러와 맞는지 협업자에게 확인하세요.",
    en: "Applied when the server is (re)started. The defaults equal both the data collector's JOINT_STIFFNESS and the server defaults. Confirm with the collaborator that they match the training-time controller.",
  },
  "ctl.imp": { ko: "관절 임피던스 J1–J7 (Nm/rad)", en: "Joint impedance J1–J7 (Nm/rad)" },
  "ctl.auto": { ko: "GUI 실행 시 서버 자동 시작", en: "Auto-start the server when the GUI launches" },
  "ctl.defaults": { ko: "기본값", en: "Defaults" },

  "conn.title": { ko: "연결", en: "Connections" },
  "conn.robot_url": { ko: "Robot Server 주소", en: "Robot Server address" },
  "conn.mode": { ko: "CaCoFlow 접속 방식", en: "CaCoFlow access" },
  "conn.mode.ssh": { ko: "내장 SSH 터널", en: "Built-in SSH tunnel" },
  "conn.mode.url": { ko: "직접 URL (자체 터널)", en: "Direct URL (own tunnel)" },
  "conn.key": { ko: "SSH 키 파일 (선택)", en: "SSH key file (optional)" },
  "conn.remote_port": { ko: "원격 정책 포트", en: "Remote policy port" },
  "conn.policy_url": { ko: "정책 서버 URL (8000 아님)", en: "Policy server URL (not 8000)" },
  "conn.zoom": { ko: "agent 카메라 zoom (기본값)", en: "Agent camera zoom (default)" },
  "conn.timeout": { ko: "요청 timeout (s)", en: "Request timeout (s)" },
  "conn.max_age": { ko: "최대 관측 나이 (s)", en: "Max observation age (s)" },
  "conn.save": { ko: "설정 저장", en: "Save settings" },
  "conn.saved": { ko: "저장됨", en: "Saved" },

  "badge.idle": { ko: "대기", en: "idle" },
  "badge.preparing": { ko: "준비 중", en: "preparing" },
  "badge.connected": { ko: "연결됨", en: "connected" },
  "badge.ready": { ko: "준비 완료", en: "ready" },
  "badge.running": { ko: "실행 중", en: "running" },
  "badge.awaiting_outcome": { ko: "결과 입력 대기", en: "awaiting outcome" },
  "badge.error": { ko: "오류", en: "error" },
  "err.trial": { ko: "시행 오류 (세션이 닫혔습니다. 새 시행으로 다시 시작하세요): ", en: "Trial error (the session is closed; start a new trial): " },
  "err.bringup": { ko: "로봇 준비 실패: ", en: "Robot bring-up failed: " },
  "err.robot": { ko: "로봇 연결 오류: ", en: "Robot connection error: " },
  "msg.need_gripper": { ko: "그리퍼 값 1의 의미를 선택하세요.", en: "Choose what gripper value 1 means." },
};

function getLang() {
  try {
    const saved = localStorage.getItem("policy-bridge-lang");
    if (saved === "ko" || saved === "en") return saved;
  } catch (e) { /* ignore */ }
  return "ko";
}

function t(key, vars) {
  const entry = I18N[key];
  if (!entry) return key;
  let s = entry[getLang()] ?? entry.ko;
  if (vars) for (const [k, v] of Object.entries(vars)) s = s.replaceAll(`{${k}}`, v);
  return s;
}

function applyStaticI18n() {
  document.querySelectorAll("[data-i18n]").forEach((el) => { el.innerHTML = t(el.getAttribute("data-i18n")); });
  document.querySelectorAll("[data-i18n-ph]").forEach((el) => { el.setAttribute("placeholder", t(el.getAttribute("data-i18n-ph"))); });
}

function setLang(lang) {
  try { localStorage.setItem("policy-bridge-lang", lang); } catch (e) { /* ignore */ }
  document.documentElement.setAttribute("lang", lang);
  applyStaticI18n();
  document.querySelectorAll(".lang-toggle button").forEach((b) => b.classList.toggle("active", b.dataset.lang === lang));
  if (typeof onLangChange === "function") onLangChange();
}
