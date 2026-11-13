#!/usr/bin/env python3
"""Franka FR3 WebSocket Robot Server (robot_server_format.json) on ws://127.0.0.1:8000.

Process layout (fixes "controller_torque_discontinuity"):
  * control process  : pylibfranka 1 kHz joint-impedance loop ONLY (own GIL, SCHED_FIFO if allowed, gc off).
                       Talks to the server through shared memory (no locks, no queues in the loop).
  * gripper process  : fr3_bridge FciGripper (pylibfranka Gripper blocks the GIL).
  * main process     : WebSocket server, JSON, RealSense cameras (threads), base64 encoding.

Cameras (RealSense, color 640x480 rgb8): agent = D455, wrist = D405 (override with --agent-serial / --wrist-serial).

usage:
  source ~/venvs/franka/bin/activate
  python robot_server.py --ip 172.16.0.2 --port 8000
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import logging
import multiprocessing as mp
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import websockets

FR3_BRIDGE_DIR = Path("/home/gibeom/fr3_bridge")
if str(FR3_BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(FR3_BRIDGE_DIR))

logger = logging.getLogger("robot_server")

GRIPPER_MAX_WIDTH = 0.07975610345602036
DEFAULT_STIFFNESS = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
DEFAULT_DAMPING = [49.0, 49.0, 49.0, 49.0, 31.6, 24.5, 14.1]
AGENT_SERIAL = "338122300664"   # RealSense D455 (external / agent view)
WRIST_SERIAL = "230422272249"   # RealSense D405 (wrist)

# control-process command / status codes
CMD_IDLE, CMD_RUN, CMD_STOP = 0, 1, 2
ST_IDLE, ST_RUNNING, ST_FAULT = 0, 1, 2


# ============================================================================
# 1 kHz control process
# ============================================================================
def _dur_ms(dur) -> float:
    """pylibfranka Duration -> milliseconds (1.0 = no missed cycle)."""
    for name in ("to_msec", "to_ms", "toMSec"):
        f = getattr(dur, name, None)
        if callable(f):
            return float(f())
    for name in ("to_sec", "toSec"):
        f = getattr(dur, name, None)
        if callable(f):
            return 1000.0 * float(f())
    try:
        return 1000.0 * float(dur)
    except Exception:  # noqa: BLE001
        return 1.0


def _control_process(ip, torque_scale, K, D, target, t_target, state, cmd, status, fault_q):
    """Runs in its own process. target = [q_des(7), dq_des(7)], state = [q(7)]."""
    import gc
    import os

    sys.path.insert(0, str(FR3_BRIDGE_DIR))
    from fr3_bridge import SafetyFilter, limits
    from fr3_bridge.fci_libfranka import FciFr3

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
        print("[control] SCHED_FIFO priority 80", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[control] could not set SCHED_FIFO ({e}); running with normal priority", flush=True)

    K = np.asarray(K, float)
    D = np.asarray(D, float)
    robot = FciFr3(ip, gripper=False)
    tgt = np.frombuffer(target, dtype=np.float64)
    st = np.frombuffer(state, dtype=np.float64)

    while True:
        if cmd.value != CMD_RUN:
            try:
                st[:7] = robot.read_state().q
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.01)
            continue

        # ------------------------------------------------------------ one trial
        safety = SafetyFilter(torque_scale=torque_scale)
        safety.dtau_max = 0.5 * limits.DTAU_MAX          # margin below libfranka's 1000 Nm/s
        max_step = limits.DQ_MAX * 0.5 * robot.dt         # slew limit on the reference (rad / ms)
        error = False
        try:
            try:
                robot.recover()
            except Exception:  # noqa: BLE001
                pass
            s = robot._start_torque()
            safety.reset()
            q_ref = s.q.copy()
            dq_ff = np.zeros(7)
            tgt[:7] = s.q
            tgt[7:] = 0.0
            t_target.value = time.monotonic()
            st[:7] = s.q
            status.value = ST_RUNNING
            gc.disable()
            import pylibfranka as plf
            n_cycles, n_missed, max_gap_ms, max_compute_ms, min_success = 0, 0, 0.0, 0.0, 1.0
            max_dtau, tau_prev = 0.0, np.zeros(7)
            t_wall_prev = time.perf_counter()
            while True:
                t_c0 = time.perf_counter()
                q_goal = tgt[:7].copy()
                dq_goal = tgt[7:].copy() if (time.monotonic() - t_target.value) < 0.025 else np.zeros(7)
                q_ref += np.clip(q_goal - q_ref, -max_step, max_step)
                dq_ff += 0.05 * (dq_goal - dq_ff)          # ~20 ms low-pass: no steps in the D term
                tau = K * (q_ref - s.q) + D * (dq_ff - s.dq) + robot.coriolis(s)
                tau = safety(tau, s, robot.dt)
                max_dtau = max(max_dtau, float(np.max(np.abs(tau - tau_prev))))
                tau_prev = tau.copy()
                cmd_t = plf.Torques(tau.tolist())
                if cmd.value == CMD_STOP:
                    cmd_t.motion_finished = True
                    robot.ctrl.writeOnce(cmd_t)
                    break
                max_compute_ms = max(max_compute_ms, 1000 * (time.perf_counter() - t_c0))
                robot.ctrl.writeOnce(cmd_t)
                raw, dur = robot.ctrl.readOnce()           # blocks until the next 1 ms robot state
                s = robot._convert(raw)
                st[:7] = s.q
                # timing diagnostics
                n_cycles += 1
                now = time.perf_counter()
                max_gap_ms = max(max_gap_ms, 1000 * (now - t_wall_prev))
                t_wall_prev = now
                d_ms = _dur_ms(dur)
                if d_ms > 1.5:
                    n_missed += 1
                min_success = min(min_success, s.success_rate)
            print(f"[control] trial ok: cycles={n_cycles} missed={n_missed} max_gap={max_gap_ms:.2f}ms "
                  f"max_compute={max_compute_ms:.3f}ms min_success_rate={min_success:.3f} "
                  f"max_dtau/cycle={max_dtau:.3f}Nm", flush=True)
        except Exception as e:  # noqa: BLE001
            import traceback
            error = True
            code = "robot_fault" if "reflex" in str(e) or "Safety" in type(e).__name__ else "controller_stopped"
            text = f"{type(e).__name__}: {e}"
            print(f"[control] FAULT {code}: {text}", flush=True)
            traceback.print_exc()
            try:
                print(f"[control] last q={np.round(s.q, 4).tolist()} dq={np.round(s.dq, 3).tolist()} "
                      f"q_ref={np.round(q_ref, 4).tolist()} tau={np.round(tau, 2).tolist()}", flush=True)
                print(f"[control] timing: cycles={n_cycles} missed(>1.5ms)={n_missed} max_gap={max_gap_ms:.2f}ms "
                      f"max_compute={max_compute_ms:.3f}ms min_success_rate={min_success:.3f} "
                      f"max_dtau/cycle={max_dtau:.3f}Nm", flush=True)
            except Exception:  # noqa: BLE001
                pass
            fault_q.value = f"{code}|{text}".encode("utf-8", "replace")[:1023]
            status.value = ST_FAULT
        finally:
            gc.enable()
            try:
                robot._stop_torque(error)
            except Exception:  # noqa: BLE001
                pass
            if not error:
                status.value = ST_IDLE
            cmd.value = CMD_IDLE


class ControlClient:
    """Main-process handle to the control process."""

    def __init__(self, ip, torque_scale, K, D):
        ctx = mp.get_context("spawn")
        self.K, self.D = list(K), list(D)
        self.target = ctx.RawArray("d", 14)
        self.t_target = ctx.RawValue("d", 0.0)
        self.state = ctx.RawArray("d", 7)
        self.cmd = ctx.RawValue("i", CMD_IDLE)
        self.status = ctx.RawValue("i", ST_IDLE)
        self.fault_q = ctx.RawArray("c", 1024)   # "code|message" of the last fault
        self.proc = ctx.Process(target=_control_process, daemon=True,
                                args=(ip, torque_scale, self.K, self.D, self.target, self.t_target,
                                      self.state, self.cmd, self.status, self.fault_q))
        self.proc.start()
        self._tgt = np.frombuffer(self.target, dtype=np.float64)
        self._st = np.frombuffer(self.state, dtype=np.float64)

    def start_trial(self, timeout=5.0):
        self.fault_q.value = b""
        self.status.value = ST_IDLE
        self.cmd.value = CMD_RUN
        t0 = time.monotonic()
        while self.status.value == ST_IDLE and time.monotonic() - t0 < timeout:
            time.sleep(0.005)
        if self.status.value != ST_RUNNING:
            raise RuntimeError(self.fault() or ("controller_stopped", "controller did not start"))

    def stop_trial(self, timeout=3.0):
        if self.status.value == ST_RUNNING:
            self.cmd.value = CMD_STOP
            t0 = time.monotonic()
            while self.cmd.value != CMD_IDLE and time.monotonic() - t0 < timeout:
                time.sleep(0.005)

    def set_target(self, q, dq):
        self._tgt[:7] = q
        self._tgt[7:] = dq
        self.t_target.value = time.monotonic()

    def joints(self):
        return self._st.copy()

    def fault(self) -> Optional[tuple[str, str]]:
        if self.status.value == ST_FAULT:
            raw = self.fault_q.value.decode("utf-8", "replace")
            code, _, msg = raw.partition("|")
            return (code or "controller_stopped", msg or "controller fault")
        return None


# ============================================================================
# Cameras
# ============================================================================
class RealSenseCamera:
    """Latest 640x480 rgb8 color frame, grabbed in a background thread."""

    def __init__(self, serial: str, name: str):
        import pyrealsense2 as rs
        self.serial, self.name, self.frame, self.t_frame = serial, name, None, 0.0
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.pipe.start(cfg)
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        t0 = time.monotonic()
        while self.frame is None and time.monotonic() - t0 < 5.0:
            time.sleep(0.01)
        if self.frame is None:
            raise RuntimeError(f"camera {name} ({serial}): no frame")
        logger.info("Camera %s (RealSense %s) streaming 640x480 rgb8", name, serial)

    def _loop(self):
        while self._run:
            try:
                f = self.pipe.wait_for_frames(1000).get_color_frame()
                if f:
                    self.frame = np.asanyarray(f.get_data()).copy()
                    self.t_frame = time.monotonic()
            except Exception as e:  # noqa: BLE001
                logger.warning("camera %s: %s", self.name, e)

    def stale(self, max_age=1.0) -> bool:
        return time.monotonic() - self.t_frame > max_age

    def close(self):
        self._run = False
        try:
            self.pipe.stop()
        except Exception:  # noqa: BLE001
            pass


def _b64(img: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(img, dtype=np.uint8).tobytes()).decode("ascii")


PREVIEW_DIR = Path(__file__).resolve().parent / "camera_preview"


def start_preview(cams: dict, period: float = 1.0) -> None:
    """Save the latest agent / wrist frames to camera_preview/{agent,wrist}.png every `period` s (open them in an
    image viewer: they refresh live). Runs whether or not a client is connected."""
    from PIL import Image
    PREVIEW_DIR.mkdir(exist_ok=True)

    def loop():
        n = 0
        while True:
            for name, cam in cams.items():
                if cam.frame is not None:
                    tmp = PREVIEW_DIR / f"{name}.tmp.png"
                    Image.fromarray(cam.frame).save(tmp)
                    tmp.replace(PREVIEW_DIR / f"{name}.png")
            n += 1
            if n % 10 == 1:
                logger.info("camera preview: agent %s, wrist %s (age %.2fs / %.2fs) -> %s",
                            None if cams["agent"].frame is None else cams["agent"].frame.shape,
                            None if cams["wrist"].frame is None else cams["wrist"].frame.shape,
                            time.monotonic() - cams["agent"].t_frame, time.monotonic() - cams["wrist"].t_frame,
                            PREVIEW_DIR)
            time.sleep(period)

    threading.Thread(target=loop, daemon=True).start()


# ============================================================================
# WebSocket server
# ============================================================================
class FrankaWebSocketServer:
    def __init__(self, ctrl: ControlClient, gripper, cams: dict, libfranka_version: str):
        self.ctrl, self.gripper, self.cams = ctrl, gripper, cams
        self.libfranka_version = libfranka_version
        self.busy = False

    def gripper_pos(self) -> float:
        w = GRIPPER_MAX_WIDTH
        if self.gripper is not None:
            try:
                w = float(self.gripper.read_width())
            except Exception:  # noqa: BLE001
                pass
        return float(np.clip(1.0 - w / GRIPPER_MAX_WIDTH, 0.0, 1.0))

    async def handle(self, ws):
        if self.busy:
            await ws.close(1013, "only one client at a time")
            return
        self.busy = True
        try:
            logger.info("Client connected: starting 1 kHz controller...")
            try:
                await asyncio.to_thread(self.ctrl.start_trial)
            except Exception as e:  # noqa: BLE001
                code, msg = e.args[0] if e.args and isinstance(e.args[0], tuple) else ("controller_stopped", str(e))
                await ws.send(json.dumps({"type": "error", "code": code, "message": msg}))
                return
            await ws.send(json.dumps({
                "type": "robot_info", "robot_model": "FR3", "libfranka_version": self.libfranka_version,
                "joint_stiffness": self.ctrl.K, "joint_damping": self.ctrl.D,
                "cameras": {"agent": self.cams["agent"].serial, "wrist": self.cams["wrist"].serial},
                "images_already_cropped": False,
            }))
            stop = asyncio.Event()
            tasks = [asyncio.create_task(self._state_loop(ws, stop)),
                     asyncio.create_task(self._image_loop(ws, stop))]
            try:
                async for raw in ws:
                    m = json.loads(raw)
                    t = m.get("type")
                    if t == "joint_target":
                        q, dq = np.asarray(m["q"], float), np.asarray(m["dq"], float)
                        if q.shape != (7,) or dq.shape != (7,) or not (np.isfinite(q).all() and np.isfinite(dq).all()):
                            await ws.send(json.dumps({"type": "error", "code": "bad_message", "message": "bad joint_target"}))
                            break
                        self.ctrl.set_target(q, dq)
                    elif t == "gripper":
                        close = bool(m["close"])
                        logger.info("gripper close=%s", close)
                        if self.gripper is not None:
                            if close:
                                self.gripper.grasp(0.0, speed=0.1, force=20.0, epsilon_inner=0.08,
                                                   epsilon_outer=0.08, wait=False)
                            else:
                                self.gripper.move(0.07976, speed=0.1, wait=False)
                    elif t == "stop":
                        logger.info("stop received")
                        break
                    else:
                        await ws.send(json.dumps({"type": "error", "code": "bad_message", "message": f"unknown type {t!r}"}))
                        break
            except websockets.ConnectionClosed:
                logger.info("connection closed")
            finally:
                stop.set()
                for tk in tasks:
                    tk.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                await asyncio.to_thread(self.ctrl.stop_trial)
                logger.info("Trial ended, controller stopped.")
        finally:
            self.busy = False

    async def _state_loop(self, ws, stop):
        while not stop.is_set():
            f = self.ctrl.fault()
            if f is not None:
                logger.error("controller fault: %s", f)
                await ws.send(json.dumps({"type": "error", "code": f[0], "message": f[1]}))
                await ws.close()
                return
            await ws.send(json.dumps({"type": "state", "joint_position": self.ctrl.joints().tolist(),
                                      "gripper_position": self.gripper_pos()}))
            await asyncio.sleep(0.01)

    async def _image_loop(self, ws, stop):
        period = 1.0 / 15.0
        nxt = time.monotonic()
        while not stop.is_set():
            for name, cam in self.cams.items():
                if cam.stale():
                    await ws.send(json.dumps({"type": "error", "code": "camera_lost",
                                              "message": f"{name} camera ({cam.serial}) gives no new frames"}))
                    await ws.close()
                    return
            a, w = self.cams["agent"].frame, self.cams["wrist"].frame
            msg = await asyncio.to_thread(lambda: json.dumps({
                "type": "images",
                "agent": {"encoding": "rgb8", "width": 640, "height": 480, "data": _b64(a)},
                "wrist": {"encoding": "rgb8", "width": 640, "height": 480, "data": _b64(w)},
            }))
            await ws.send(msg)
            sent = getattr(self, "_img_sent", 0) + 1
            self._img_sent = sent
            if sent == 1 or sent % 150 == 0:
                logger.info("images sent to client: %d messages (%.1f MB each)", sent, len(msg) / 1e6)
            nxt += period
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))


async def serve(args):
    from fr3_bridge.gripper import FciGripper
    try:
        import pylibfranka
        ver = getattr(pylibfranka, "__version__", "0.21.2")
    except Exception:  # noqa: BLE001
        ver = "0.21.2"

    cams = {"agent": RealSenseCamera(args.agent_serial, "agent"),
            "wrist": RealSenseCamera(args.wrist_serial, "wrist")}
    start_preview(cams)
    ctrl = ControlClient(args.ip, args.torque_scale, DEFAULT_STIFFNESS, DEFAULT_DAMPING)
    gripper = None if args.no_gripper else FciGripper(args.ip)
    srv = FrankaWebSocketServer(ctrl, gripper, cams, ver)
    try:
        async with websockets.serve(srv.handle, args.host, args.port, max_size=None, compression=None,
                                    ping_interval=20, ping_timeout=20):
            logger.info("Franka Robot Server on ws://%s:%d (FCI %s, torque_scale %.2f)",
                        args.host, args.port, args.ip, args.torque_scale)
            await asyncio.Future()
    finally:
        ctrl.stop_trial()
        for c in cams.values():
            c.close()
        if gripper is not None:
            gripper.close()


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    p = argparse.ArgumentParser(description="Franka FR3 WebSocket Robot Server")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--ip", default="172.16.0.2", help="Franka FCI IP")
    p.add_argument("--torque-scale", type=float, default=0.5)
    p.add_argument("--agent-serial", default=AGENT_SERIAL, help="RealSense serial of the agent (external) camera")
    p.add_argument("--wrist-serial", default=WRIST_SERIAL, help="RealSense serial of the wrist camera")
    p.add_argument("--no-gripper", action="store_true")
    args = p.parse_args()
    try:
        asyncio.run(serve(args))
    except KeyboardInterrupt:
        logger.info("stopped by user")


if __name__ == "__main__":
    main()
