#!/usr/bin/env python3
"""Web dashboard for CaCoFlow trials (see CACOFLOW.md, "GUI").

Wraps cacoflow_bridge.py / cacoflow_client.py (unchanged): Robot Server start/stop with the controller
parameters, robot connection + camera preview, CaCoFlow server connection (SSH tunnel), trial start/stop
and the operator's outcome report. Run it with ./run_cacoflow_gui.sh (binds 127.0.0.1 only).

Safety notes:
  * Connecting to the Robot Server starts its 1 kHz controller (the arm is then held under impedance control).
  * Stop = hold the current position and end the trial. The physical e-stop is the real safety device.
  * The CaCoFlow token and SSH password live in this process's memory only (never written to disk).
"""
from __future__ import annotations

import io
import json
import logging
import os
import signal
import socket
import subprocess
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal, Optional

import numpy as np
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from PIL import Image
from pydantic import BaseModel, Field

from cacoflow_bridge import WsRobot
from cacoflow_client import InferenceClient, run_trial
from robot_client2 import SSHTunnel

logger = logging.getLogger("cacoflow_gui")
HERE = Path(__file__).resolve().parent
STATIC = HERE / "gui_static"
CONFIG_PATH = Path.home() / ".config" / "policy_bridge" / "gui_config.json"
SERVER_LOG = HERE / "server.log"
ROBOT_HOST, ROBOT_PORT = "127.0.0.1", 8000

# Same values as robot_server.py's defaults (and the data collector's JOINT_STIFFNESS).
DEFAULT_IMPEDANCE = [3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]


# ------------------------------------------------------------------------------------------ config
class Config(BaseModel):
    robot_url: str = Field("ws://127.0.0.1:8000", pattern=r"^ws://(127\.0\.0\.1|localhost)(:\d+)?$")
    policy_mode: Literal["ssh", "url"] = "ssh"
    ssh_host: str = Field("155.230.12.123", max_length=253)
    ssh_port: int = Field(22, ge=1, le=65535)
    ssh_user: str = Field("tanvir", max_length=64)
    ssh_key: str = Field("", max_length=500)
    remote_policy_port: int = Field(8000, ge=1, le=65535)
    policy_url: str = Field("http://127.0.0.1:8100", max_length=300)
    agent_zoom: float = Field(1.0, ge=1.0, le=3.0)
    timeout: float = Field(5.0, gt=0, le=60)
    max_observation_age: float = Field(0.5, gt=0, le=5)
    # Robot Server controller (applied when the server is (re)started)
    joint_impedance: list[float] = Field(default_factory=lambda: list(DEFAULT_IMPEDANCE), min_length=7, max_length=7)
    v_max: float = Field(0.8, gt=0, le=1.5)
    a_max: float = Field(2.5, gt=0, le=5.0)
    j_max: float = Field(1500.0, gt=0, le=3000.0)
    filter_wn: float = Field(8.0, ge=1.0, le=20.0)
    auto_start_server: bool = True

    def server_args(self) -> list[str]:
        return ["--joint-impedance", *[f"{k:g}" for k in self.joint_impedance], "--v-max", f"{self.v_max:g}",
                "--a-max", f"{self.a_max:g}", "--j-max", f"{self.j_max:g}", "--filter-wn", f"{self.filter_wn:g}"]

    def controller_key(self) -> list:
        return [*self.joint_impedance, self.v_max, self.a_max, self.j_max, self.filter_wn]


def validate_impedance(cfg: Config) -> None:
    if any(not (0 < k <= 4000) for k in cfg.joint_impedance):
        raise HTTPException(400, "joint impedance must be in (0, 4000] Nm/rad")


def load_config() -> Config:
    try:
        return Config(**json.loads(CONFIG_PATH.read_text()))
    except (OSError, ValueError):
        return Config()


def save_config(cfg: Config) -> None:
    CONFIG_PATH.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    tmp = CONFIG_PATH.with_suffix(".tmp")
    tmp.write_text(cfg.model_dump_json(indent=2))
    os.replace(tmp, CONFIG_PATH)


# ------------------------------------------------------------------------------- Robot Server process
def port_open(host: str = ROBOT_HOST, port: int = ROBOT_PORT) -> bool:
    """TCP probe only: a plain connect does not start the controller (that needs a WebSocket handshake)."""
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


def server_pids() -> list[int]:
    pids = []
    for p in Path("/proc").iterdir():
        if not p.name.isdigit():
            continue
        try:
            cmd = (p / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if cmd and Path(cmd[0].decode(errors="ignore")).name.startswith("python") and \
                any(Path(c.decode(errors="ignore")).name == "robot_server.py" for c in cmd[1:3]):
            pids.append(int(p.name))
    return pids


def tail(path: Path, n: int = 40, max_bytes: int = 16384) -> list[str]:
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - max_bytes))
            return f.read().decode(errors="replace").splitlines()[-n:]
    except OSError:
        return []


def pin_away_from(cpu: int) -> None:
    """Keep this process's threads off the robot's 1 kHz control CPU (like run_cacoflow.sh does)."""
    try:
        cpus = set(range(os.cpu_count() or 1)) - {cpu}
        if not cpus:
            return
        for tid in os.listdir("/proc/self/task"):
            try:
                os.sched_setaffinity(int(tid), cpus)
            except OSError:
                pass
        logger.info("pinned GUI threads away from control CPU %d", cpu)
    except Exception as exc:  # noqa: BLE001
        logger.warning("could not pin GUI threads: %s", exc)


class ServerManager:
    def __init__(self) -> None:
        self.proc: Optional[subprocess.Popen] = None
        self.started_at = 0.0
        self.applied: Optional[list] = None        # controller parameters of the server we started
        self.message = ""

    def state(self) -> str:
        if port_open():
            return "running"
        if self.proc is not None and self.proc.poll() is None:
            return "starting"
        return "stopped"

    def start(self, cfg: Config) -> None:
        if self.state() != "stopped" or server_pids():
            raise HTTPException(409, "Robot Server is already running or starting")
        if subprocess.run(["sudo", "-n", "true"], stdin=subprocess.DEVNULL, capture_output=True).returncode != 0:
            raise HTTPException(409, "sudo password needed for setup_rt.sh: run `sudo -v` in the terminal that "
                                     "started the GUI (run_cacoflow_gui.sh does this at launch), then retry")
        validate_impedance(cfg)
        if not (HERE / "run_server.sh").is_file():
            raise HTTPException(409, "run_server.sh not found: this copy has no Robot Server; "
                                     "start the Robot Server on the robot PC yourself")
        self.started_at = time.time()
        self.applied = cfg.controller_key()
        self.message = ""
        env = {k: v for k, v in os.environ.items() if k not in ("VIRTUAL_ENV", "PYTHONHOME")}
        out = open(HERE / "gui_server_start.log", "ab")
        self.proc = subprocess.Popen([str(HERE / "run_server.sh"), *cfg.server_args()], cwd=HERE, env=env,
                                     stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT)
        logger.info("started run_server.sh (pid %d) args=%s", self.proc.pid, cfg.server_args())
        threading.Thread(target=self._after_start, daemon=True).start()

    def _after_start(self) -> None:
        proc = self.proc
        while proc is not None and proc.poll() is None and not port_open():
            time.sleep(0.5)
        if port_open():
            try:
                cpu = int((HERE / ".control_cpu").read_text().strip())
                pin_away_from(cpu)
            except (OSError, ValueError):
                pass
        elif proc is not None:
            self.message = f"run_server.sh exited with code {proc.returncode}; see the log below"

    def stop(self) -> None:
        """SIGINT like run_server.sh does, so the controller brakes and stops cleanly."""
        pids = server_pids()
        for pid in pids:
            os.kill(pid, signal.SIGINT)
        deadline = time.time() + 12
        while time.time() < deadline and (server_pids() or port_open()):
            time.sleep(0.2)
        if server_pids():
            raise HTTPException(500, "Robot Server did not exit within 12 s (use the terminal)")


# --------------------------------------------------------------------------------------- trial session
class TrialParams(BaseModel):
    instruction: str = Field(min_length=1, max_length=300)
    gripper_one_means: Literal["closed", "open"]
    trial_id: Optional[str] = Field(None, max_length=100)
    placement: Optional[str] = Field(None, max_length=100)
    appearance: Literal["seen", "unseen"] = "seen"
    max_steps: int = Field(60, ge=1, le=6000)
    agent_zoom: float = Field(1.0, ge=1.0, le=3.0)
    estop_ready: bool


class Outcome(BaseModel):
    success: Optional[bool] = None
    wrong_object: Optional[bool] = None
    notes: str = Field("", max_length=2000)


class ConnectPolicy(BaseModel):
    token: str = Field("", max_length=500)
    ssh_password: str = Field("", max_length=200)


def jpeg(arr: np.ndarray, width: int = 480) -> bytes:
    img = Image.fromarray(arr)
    img = img.resize((width, int(arr.shape[0] * width / arr.shape[1])))
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=70)
    return buf.getvalue()


class Session:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.cfg = load_config()
        self.server = ServerManager()
        self.tunnel: Optional[SSHTunnel] = None
        self.client: Optional[InferenceClient] = None
        self.robot: Optional[WsRobot] = None
        self.busy: Optional[str] = None                 # a long blocking operation in progress
        self.bringup: dict = {"step": "idle", "message": ""}
        self.trial_state = "idle"                       # idle | running | awaiting_outcome | error
        self.trial_error = ""
        self.trial_meta: dict = {}
        self.progress: dict = {}
        self.result: Optional[dict] = None
        self.stop_flag = threading.Event()
        self.thread: Optional[threading.Thread] = None

    # --- Robot Server + robot connection ("bring-up") ---
    def start_bringup(self) -> None:
        with self.lock:
            if self.busy or self.trial_state == "running":
                raise HTTPException(409, f"busy: {self.busy or 'trial running'}")
            if self.robot is not None and self.robot._error is None:
                raise HTTPException(409, "robot already connected")
            self.busy = "bringup"
            self.bringup = {"step": "starting", "message": ""}
        threading.Thread(target=self._bringup, daemon=True).start()

    def _bringup(self) -> None:
        try:
            if self.robot is not None:                  # dead connection from before
                self._close_robot()
            if self.server.state() == "stopped":
                self.bringup = {"step": "server", "message": "Robot Server"}
                self.server.start(self.cfg)
            deadline = time.time() + 240                # RT setup + camera reset take a while
            while self.server.state() != "running":
                if self.server.state() == "stopped":
                    raise RuntimeError(self.server.message or "Robot Server stopped")
                if time.time() > deadline:
                    raise RuntimeError("Robot Server did not come up within 240 s")
                time.sleep(0.5)
            self.bringup = {"step": "robot", "message": ""}
            self.robot = WsRobot(self.cfg.robot_url, "closed", self.cfg.agent_zoom)
            self.bringup = {"step": "done", "message": ""}
        except HTTPException as exc:
            self.bringup = {"step": "error", "message": str(exc.detail)}
        except Exception as exc:  # noqa: BLE001
            logger.exception("bring-up failed")
            self.bringup = {"step": "error", "message": f"{type(exc).__name__}: {exc}"}
        finally:
            self.busy = None

    def _close_robot(self) -> None:
        robot, self.robot = self.robot, None
        if robot is not None:
            robot.close()                               # sends "stop": the Robot Server stops its controller

    def disconnect_robot(self) -> None:
        with self.lock:
            if self.trial_state == "running" or self.busy:
                raise HTTPException(409, "trial or connection in progress")
            self._close_robot()
            self.bringup = {"step": "idle", "message": ""}

    # --- CaCoFlow server ---
    def connect_policy(self, body: ConnectPolicy) -> dict:
        with self.lock:
            if self.busy or self.trial_state == "running":
                raise HTTPException(409, f"busy: {self.busy or 'trial running'}")
            self.busy = "policy"
        try:
            self.disconnect_policy(force=True)
            token = body.token or os.environ.get("CACOFLOW_API_TOKEN", "")
            if not token:
                raise HTTPException(400, "CaCoFlow API token required")
            cfg, tunnel = self.cfg, None
            try:
                if cfg.policy_mode == "ssh":
                    # "" (not None): a failed key login must raise instead of prompting on the terminal
                    tunnel = SSHTunnel(cfg.ssh_host, cfg.ssh_port, cfg.ssh_user, ssh_password=body.ssh_password or "",
                                       ssh_key=cfg.ssh_key or None, remote_host="127.0.0.1",
                                       remote_port=cfg.remote_policy_port)
                    url = f"http://127.0.0.1:{tunnel.start()}"
                else:
                    url = cfg.policy_url
                client = InferenceClient(url, token, cfg.timeout, cfg.max_observation_age)
            except HTTPException:
                raise
            except Exception as exc:  # noqa: BLE001
                if tunnel is not None:
                    tunnel.close()
                raise HTTPException(502, f"{type(exc).__name__}: {exc}") from exc
            self.tunnel, self.client = tunnel, client
            return client.info
        finally:
            self.busy = None

    def disconnect_policy(self, force: bool = False) -> None:
        with self.lock:
            if not force and (self.busy or self.trial_state == "running"):
                raise HTTPException(409, "trial or connection in progress")
            self.client = None
            if self.tunnel is not None:
                self.tunnel.close()
                self.tunnel = None
            if self.trial_state in ("awaiting_outcome", "error"):
                self.trial_state = "idle"

    # --- trial ---
    def start_trial(self, p: TrialParams) -> None:
        with self.lock:
            if self.trial_state == "running":
                raise HTTPException(409, "a trial is already running")
            if self.trial_state == "awaiting_outcome":
                raise HTTPException(409, "report the previous trial's outcome first")
            if self.busy:
                raise HTTPException(409, f"busy: {self.busy}")
            client, robot = self.client, self.robot
            if client is None:
                raise HTTPException(409, "connect to the CaCoFlow server first")
            if robot is None or robot._error is not None:
                raise HTTPException(409, "robot not connected")
            if not p.estop_ready:
                raise HTTPException(400, "confirm that the e-stop is within reach")
            info = client.info
            if info.get("mock"):
                raise HTTPException(400, "server is in mock mode")
            if list(info.get("action_shape", [])) != [10, 8]:
                raise HTTPException(400, f"unexpected action_shape {info.get('action_shape')}")
            if p.instruction not in info.get("tasks", []):
                raise HTTPException(400, "instruction is not one of the server's tasks")
            self.trial_state, self.trial_error, self.result = "running", "", None
            self.trial_meta = {"instruction": p.instruction, "trial_id": p.trial_id or None,
                               "placement": p.placement or None, "appearance": p.appearance,
                               "gripper_one_means": p.gripper_one_means, "max_steps": p.max_steps}
            self.progress = {"step": 0, "max_steps": p.max_steps, "chunks": 0, "last_rt_ms": None,
                             "rt_ms": [], "started": time.time(), "stopped": False}
            self.stop_flag.clear()
            self.thread = threading.Thread(target=self._run_trial, args=(p, client, robot), daemon=True)
            self.thread.start()

    def _run_trial(self, p: TrialParams, client: InferenceClient, robot: WsRobot) -> None:
        orig_predict = client.predict
        try:
            robot.one_is_closed = p.gripper_one_means == "closed"
            robot.agent_zoom = p.agent_zoom
            robot._last_close = None
            t0 = time.monotonic()                       # make sure the first frame already has the new zoom
            deadline = t0 + 1.0
            while robot._t_images <= t0 and time.monotonic() < deadline:
                time.sleep(0.01)

            def predict(*a, **k):
                if self.stop_flag.is_set():
                    raise KeyboardInterrupt
                actions, reply = orig_predict(*a, **k)
                rt = reply["client_roundtrip_ms"]
                self.progress["chunks"] += 1
                self.progress["last_rt_ms"] = rt
                self.progress["rt_ms"].append(rt)
                return actions, reply

            def on_step(step, k, row, target, gripper):
                self.progress.update(step=step + 1, gripper=gripper)
                if self.stop_flag.is_set():
                    raise KeyboardInterrupt             # run_trial turns this into "operator stop" + hold

            client.predict = predict
            res = run_trial(client, robot, p.instruction, trial_id=p.trial_id or None, placement=p.placement or None,
                            appearance=p.appearance, max_steps=p.max_steps, on_step=on_step)
            self.result = {"steps": res["steps"], "chunks": res["chunks"], "clipped_steps": res["clipped_steps"]}
            self.progress["stopped"] = self.stop_flag.is_set()
            self.trial_state = "awaiting_outcome" if client.session_id is not None else "error"
            if self.trial_state == "error":
                self.trial_error = "session closed by the client"
        except Exception as exc:  # noqa: BLE001
            logger.exception("trial failed")
            robot.hold()
            self.trial_error = f"{type(exc).__name__}: {exc}"
            self.trial_state = "error"
        finally:
            client.__dict__.pop("predict", None)
            self.progress["ended"] = time.time()

    def stop_trial(self) -> None:
        with self.lock:
            if self.trial_state != "running":
                raise HTTPException(409, "no trial running")
            self.stop_flag.set()
            if self.robot is not None:
                self.robot.hold()                       # immediate hold; run_trial holds again when it unwinds

    def report_outcome(self, o: Outcome) -> dict:
        with self.lock:
            if self.trial_state != "awaiting_outcome" or self.client is None:
                raise HTTPException(409, "no trial waiting for an outcome")
            try:
                end = self.client.end_trial(success=o.success, wrong_object=o.wrong_object, notes=o.notes)
            except Exception as exc:  # noqa: BLE001
                raise HTTPException(502, f"{type(exc).__name__}: {exc}") from exc
            self.trial_state = "idle"
            return {"trial_dir": end.get("trial_dir")}

    def dismiss_error(self) -> None:
        with self.lock:
            if self.trial_state == "error":
                self.trial_state, self.trial_error = "idle", ""

    # --- state snapshot ---
    def robot_snapshot(self) -> dict:
        r = self.robot
        if r is None:
            return {"connected": False}
        if r._error is not None:
            return {"connected": False, "error": r._error}
        with r._lock:
            q, g, t_state, t_img = r._q, r._g, r._t_state, r._t_images
        now = time.monotonic()
        return {"connected": True, "q": None if q is None else [round(float(v), 3) for v in q],
                "gripper_closed": round(float(g), 2), "state_age_ms": round((now - t_state) * 1000),
                "image_age_ms": round((now - t_img) * 1000)}

    def snapshot(self) -> dict:
        rt = self.progress.get("rt_ms") or []
        prog = {k: v for k, v in self.progress.items() if k not in ("rt_ms", "started", "ended")}
        if rt:
            prog["rt_median_ms"] = round(float(np.median(rt)))
            prog["rt_max_ms"] = round(max(rt))
        if self.progress.get("started"):
            prog["elapsed_s"] = round(self.progress.get("ended", time.time()) - self.progress["started"], 1)
        srv = self.server.state()
        return {
            "server": {"state": srv, "pids": server_pids(), "message": self.server.message,
                       "restart_needed": srv == "running" and self.server.applied is not None
                       and self.server.applied != self.cfg.controller_key(),
                       "log": tail(SERVER_LOG, 25)},
            "bringup": self.bringup, "busy": self.busy,
            "robot": self.robot_snapshot(),
            "policy": {"connected": self.client is not None,
                       "info": self.client.info if self.client is not None else None},
            "trial": {"state": self.trial_state, "error": self.trial_error, "meta": self.trial_meta,
                      "progress": prog, "result": self.result},
            "token_in_env": bool(os.environ.get("CACOFLOW_API_TOKEN")),
        }

    def shutdown(self) -> None:
        try:
            if self.trial_state == "running":
                self.stop_flag.set()
                if self.thread is not None:
                    self.thread.join(timeout=8)
            if self.robot is not None:
                self.robot.hold()
                self._close_robot()
            if self.tunnel is not None:
                self.tunnel.close()
        except Exception:  # noqa: BLE001
            logger.exception("shutdown")


S = Session()


@asynccontextmanager
async def lifespan(app: FastAPI):
    if os.environ.get("CACOFLOW_GUI_AUTOSTART") == "1" and S.cfg.auto_start_server:
        if S.server.state() == "stopped" and not server_pids():
            try:
                S.server.start(S.cfg)
            except HTTPException as exc:
                S.server.message = str(exc.detail)
        elif S.server.state() == "running":
            try:
                pin_away_from(int((HERE / ".control_cpu").read_text().strip()))
            except (OSError, ValueError):
                pass
    yield
    S.shutdown()


app = FastAPI(title="policy_bridge CaCoFlow GUI", lifespan=lifespan)


@app.middleware("http")
async def local_only(request: Request, call_next):
    """DNS-rebinding / cross-site protection: this page controls a robot."""
    host = (request.headers.get("host") or "").rsplit(":", 1)[0].strip("[]")
    if host not in ("127.0.0.1", "localhost"):
        return JSONResponse({"detail": "bad Host header"}, status_code=403)
    if request.method not in ("GET", "HEAD", "OPTIONS"):
        origin = request.headers.get("origin")
        if origin is not None and origin.split("://", 1)[-1] != request.headers.get("host"):
            return JSONResponse({"detail": "cross-origin request refused"}, status_code=403)
    return await call_next(request)


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC / "index.html")


@app.get("/api/state")
def api_state() -> dict:
    return S.snapshot()


@app.get("/api/config")
def api_get_config() -> dict:
    return S.cfg.model_dump()


@app.post("/api/config")
def api_set_config(cfg: Config) -> dict:
    validate_impedance(cfg)
    with S.lock:
        S.cfg = cfg
        save_config(cfg)
    return cfg.model_dump()


@app.post("/api/robot/prepare")
def api_prepare() -> dict:
    S.start_bringup()
    return {"ok": True}


@app.post("/api/robot/disconnect")
def api_robot_disconnect() -> dict:
    S.disconnect_robot()
    return {"ok": True}


@app.post("/api/server/stop")
def api_server_stop() -> dict:
    with S.lock:
        if S.robot is not None or S.trial_state == "running" or S.busy:
            raise HTTPException(409, "disconnect the robot first")
        S.server.stop()
    return {"ok": True}


@app.post("/api/policy/connect")
def api_policy_connect(body: ConnectPolicy) -> dict:
    return S.connect_policy(body)


@app.post("/api/policy/disconnect")
def api_policy_disconnect() -> dict:
    S.disconnect_policy()
    return {"ok": True}


@app.post("/api/trial/start")
def api_trial_start(p: TrialParams) -> dict:
    S.start_trial(p)
    return {"ok": True}


@app.post("/api/trial/stop")
def api_trial_stop() -> dict:
    S.stop_trial()
    return {"ok": True}


@app.post("/api/trial/outcome")
def api_trial_outcome(o: Outcome) -> dict:
    return S.report_outcome(o)


@app.post("/api/trial/dismiss")
def api_trial_dismiss() -> dict:
    S.dismiss_error()
    return {"ok": True}


@app.get("/api/camera/{role}.jpg")
def api_camera(role: Literal["agent", "wrist"]) -> Response:
    r = S.robot
    if r is None or r._error is not None:
        raise HTTPException(404, "robot not connected")
    with r._lock:
        arr = r._agent if role == "agent" else r._wrist
    if arr is None:
        raise HTTPException(404, "no frame yet")
    return Response(jpeg(arr), media_type="image/jpeg", headers={"Cache-Control": "no-store"})


app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")


def main() -> None:
    import argparse

    import uvicorn
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
