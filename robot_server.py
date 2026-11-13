#!/usr/bin/env python3
"""Franka FR3 WebSocket Robot Server (robot_server_format.json) on ws://127.0.0.1:8000.

Process layout (pattern of github.com/gibeom25/franka-deploy, validated on this FR3):
  * control process  : pylibfranka start_joint_position_control(JointImpedance) ONLY. The robot's internal
                       joint-impedance controller computes torque; Python streams jerk/accel/velocity-limited joint
                       positions (reference_filter.py). Own GIL, SCHED_FIFO, gc off, shared memory, no locks.
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
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import websockets

FR3_BRIDGE_DIR = Path(os.environ.get("FR3_BRIDGE_DIR", str(Path.home() / "fr3_bridge")))
if str(FR3_BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(FR3_BRIDGE_DIR))

logger = logging.getLogger("robot_server")

GRIPPER_MAX_WIDTH = 0.07975610345602036
AGENT_SERIAL = "338122300664"   # RealSense D455 (external / agent view)
WRIST_SERIAL = "230422272249"   # RealSense D405 (wrist)

# Safety values from github.com/gibeom25/franka-deploy (safety/limits.py), tuned live on this FR3.
FR3_COLLISION_TORQUE = [100.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0]
FR3_COLLISION_FORCE = 100.0
DEFAULT_JOINT_IMPEDANCE = [3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]
DEFAULT_V_MAX, DEFAULT_A_MAX, DEFAULT_J_MAX, DEFAULT_FILTER_WN = 0.8, 2.5, 1500.0, 8.0
FR3_Q_LOWER = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
FR3_Q_UPPER = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])

# control-process command / status codes
CMD_IDLE, CMD_RUN, CMD_STOP = 0, 1, 2
ST_IDLE, ST_RUNNING, ST_FAULT = 0, 1, 2


# ============================================================================
# 1 kHz control process
# ============================================================================
def _dur_ms(dur) -> float:
    """pylibfranka Duration -> milliseconds (1.0 = no missed cycle)."""
    f = getattr(dur, "to_sec", None)
    if callable(f):
        return 1000.0 * float(f())
    try:
        return 1000.0 * float(dur)
    except Exception:  # noqa: BLE001
        return 1.0


def _dump_flight_recorder(rec, rec_i, rec_n, phase, text) -> None:
    """Save the last control ticks before a fault and print what they show."""
    n = min(rec_i, rec_n)
    if n < 3:
        print(f"[control] flight recorder: only {n} ticks recorded", flush=True)
        return
    idx = np.arange(rec_i - n, rec_i) % rec_n
    d = rec[idx].copy()
    out_dir = Path(__file__).resolve().parent / "fault_dumps"
    out_dir.mkdir(exist_ok=True)
    path = out_dir / time.strftime("fault_%Y%m%d_%H%M%S.npz")
    np.savez(path, rec=d, phase=phase, text=text,
             columns="t,d_ms,success,q[7],dq[7],q_d[7],target[7],q_cmd[7],qd_cmd[7],acc_cmd[7]")
    d_ms, succ = d[:, 1], d[:, 2]
    q, dq, q_d, tg, qc, vc, ac = (d[:, 3 + 7 * k: 10 + 7 * k] for k in range(7))
    dt = np.clip(d_ms, 1.0, 5.0) * 1e-3
    jerk = np.abs(np.diff(ac, axis=0)) / dt[1:, None]                 # jerk of what WE commanded
    # q_d is the robot's own desired position; it should equal our previous command if every packet arrived
    qd_vs_cmd = np.abs(q_d[1:] - qc[:-1]).max(axis=1)
    k_j = int(np.argmax(jerk.max(axis=1)))
    print(f"[control] flight recorder ({n} ticks = {d[-1, 0] - d[0, 0]:.3f}s) saved to {path}", flush=True)
    print(f"[control]   period: max {d_ms.max():.2f} ms, ticks > 1.5 ms: {(d_ms > 1.5).sum()} "
          f"(last 20 ticks: {np.round(d_ms[-20:], 1).tolist()})", flush=True)
    print(f"[control]   success_rate: min {succ.min():.3f}, last {succ[-1]:.3f}", flush=True)
    print(f"[control]   commanded: |v|max {np.abs(vc).max():.3f} rad/s, |a|max {np.abs(ac).max():.3f} rad/s^2, "
          f"jerk max {jerk.max():.0f} rad/s^3 (joint {int(np.argmax(jerk[k_j])) + 1}, {n - 1 - k_j} ticks before fault)",
          flush=True)
    print(f"[control]   robot q_d vs our previous command: max {qd_vs_cmd.max():.2e} rad "
          f"(last 5: {[f'{x:.1e}' for x in qd_vs_cmd[-5:]]})", flush=True)
    print(f"[control]   measured: |dq|max {np.abs(dq).max():.3f} rad/s, tracking |q - q_cmd|max "
          f"{np.abs(q - qc).max():.4f} rad, target-q_cmd max {np.abs(tg - qc).max():.4f} rad", flush=True)


def _control_process(ip, impedance, v_max, a_max, j_max, wn, target, t_target, state, cmd, status, fault_buf,
                     control_cpu=None):
    """Runs in its own process. Pattern from github.com/gibeom25/franka-deploy (fr3_runtime.py), validated on this
    FR3: start_joint_position_control(JointImpedance) -> the ROBOT's internal 1 kHz joint-impedance controller
    computes the torque; Python only streams joint positions shaped by a jerk/accel/velocity-limited reference
    filter. No Python-side torque => no controller_torque_discontinuity.

    target = [q_des(7), dq_des(7)] (dq ignored: the filter generates velocity), state = [q(7)]."""
    import gc
    import os
    import traceback

    import pylibfranka as pf
    from reference_filter import JerkLimitedReferenceFilter

    if control_cpu is not None:
        # Same CPU as the NIC IRQ / ksoftirqd: the robot packet wakes this process locally (no cross-CPU wake-up,
        # no idle-state exit on another core). Everything else is kept off this CPU by run_server.sh.
        try:
            os.sched_setaffinity(0, {int(control_cpu)})
            print(f"[control] pinned to CPU {control_cpu}", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[control] could not pin to CPU {control_cpu} ({e})", flush=True)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
        print("[control] SCHED_FIFO priority 80", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[control] could not set SCHED_FIFO ({e}); running with normal priority", flush=True)

    robot = pf.Robot(ip, pf.RealtimeConfig.kEnforce)
    try:
        robot.automatic_error_recovery()
    except Exception as e:  # noqa: BLE001
        print(f"[control] automatic error recovery: {e}", flush=True)
    robot.set_collision_behavior(FR3_COLLISION_TORQUE, FR3_COLLISION_TORQUE,
                                 [FR3_COLLISION_FORCE] * 6, [FR3_COLLISION_FORCE] * 6)
    robot.set_joint_impedance(list(impedance))
    filt = JerkLimitedReferenceFilter(n_dims=7, dt=1e-3, v_max=v_max, a_max=a_max, j_max=j_max, wn=wn)
    print(f"[control] connected (joint impedance {list(impedance)}, v_max {v_max}, a_max {a_max}, "
          f"j_max {j_max}, wn {wn})", flush=True)

    tgt = np.frombuffer(target, dtype=np.float64)
    st = np.frombuffer(state, dtype=np.float64)

    # Flight recorder: the last REC_N control ticks, dumped to fault_dumps/ when a fault happens.
    # columns: t, d_ms, success_rate, q(7), dq(7), q_d(7), target(7), q_cmd(7), qd_cmd(7), acc_cmd(7)
    REC_N = 500
    rec = np.zeros((REC_N, 3 + 7 * 7))
    rec_i = 0

    while True:
        if cmd.value != CMD_RUN:
            try:
                st[:7] = robot.read_once().q
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.01)
            continue

        # ------------------------------------------------------------ one trial
        error = False
        ctrl = None
        phase = "start"
        n_cycles, n_missed, max_dur_ms, min_success = 0, 0, 0.0, 1.0
        rec[:] = 0.0
        rec_i = 0
        t_trial0 = time.monotonic()
        try:
            try:
                robot.automatic_error_recovery()
            except Exception:  # noqa: BLE001
                pass
            ctrl = robot.start_joint_position_control(pf.ControllerMode.JointImpedance)
            s, _ = ctrl.readOnce()
            q = np.asarray(s.q, float)
            q_d = np.asarray(s.q_d, float)                 # latch to the robot's own q_d (no phantom velocity step)
            gap = float(np.abs(q_d - q).max())
            if gap > 0.05:
                raise RuntimeError(f"stale desired pose: max|q - q_d| = {gap:.3f} rad; run error recovery in Desk")
            filt.reset(q_d)
            tgt[:7] = q_d
            tgt[7:] = 0.0
            t_target.value = time.monotonic()
            st[:7] = q
            ctrl.writeOnce(pf.JointPositions(list(filt.q_cmd)))
            status.value = ST_RUNNING
            gc.disable()
            phase = "moving"
            stopping, t_stop = False, 0.0
            while True:
                s, dur = ctrl.readOnce()
                q = np.asarray(s.q, float)
                st[:7] = q
                n_cycles += 1
                d_ms = _dur_ms(dur)
                max_dur_ms = max(max_dur_ms, d_ms)
                if d_ms > 1.5:
                    n_missed += 1
                min_success = min(min_success, float(s.control_command_success_rate))

                if cmd.value == CMD_STOP and not stopping:
                    # Smoothly brake the reference filter itself (err=0 -> acc_target = -kd * qd_cmd)
                    # so both qd_cmd and _acc_prev decay to 0 before motion_finished=True.
                    stopping, t_stop = True, time.monotonic()
                    phase = "stopping"
                goal = filt.q_cmd.copy() if stopping else np.clip(tgt[:7], FR3_Q_LOWER + 0.05, FR3_Q_UPPER - 0.05)
                # Advance by the REAL period (1 ms normally, 2+ ms after a missed cycle), like libfranka's examples
                # (time += period). A fixed 1 ms step after a missed cycle = acceleration_discontinuity reflex.
                q_cmd = filt.step(goal, dt=min(max(d_ms, 1.0), 5.0) * 1e-3)
                finish = stopping and (
                    (
                        np.abs(np.asarray(s.dq)).max() < 0.01
                        and np.abs(filt.qd_cmd).max() < 1e-4
                        and np.abs(filt._acc_prev).max() < 1e-2
                    )
                    or time.monotonic() - t_stop > 3.0
                )
                r = rec[rec_i % REC_N]
                r[0] = time.monotonic() - t_trial0
                r[1] = d_ms
                r[2] = s.control_command_success_rate
                r[3:10] = q
                r[10:17] = s.dq
                r[17:24] = s.q_d
                r[24:31] = goal
                r[31:38] = q_cmd
                r[38:45] = filt.qd_cmd
                r[45:52] = filt._acc_prev
                rec_i += 1
                c = pf.JointPositions(list(q_cmd))
                c.motion_finished = bool(finish)
                ctrl.writeOnce(c)
                if finish:
                    break
            print(f"[control] trial ok: cycles={n_cycles} missed={n_missed} max_period={max_dur_ms:.1f}ms "
                  f"min_success_rate={min_success:.3f}", flush=True)
        except Exception as e:  # noqa: BLE001
            error = True
            code = "robot_fault" if "reflex" in str(e) else "controller_stopped"
            text = f"{type(e).__name__}: {e}"
            print(f"[control] FAULT {code} during '{phase}': {text}", flush=True)
            traceback.print_exc()
            print(f"[control] timing: cycles={n_cycles} missed_cycles={n_missed} max_period={max_dur_ms:.1f}ms "
                  f"min_success_rate={min_success:.3f}", flush=True)
            try:
                print(f"[control] filter: |qd_cmd|max={np.abs(filt.qd_cmd).max():.3f} rad/s "
                      f"|acc|max={np.abs(filt._acc_prev).max():.3f} rad/s^2 "
                      f"target-q_cmd max={np.abs(tgt[:7] - filt.q_cmd).max():.4f} rad", flush=True)
            except Exception:  # noqa: BLE001
                pass
            try:
                _dump_flight_recorder(rec, rec_i, REC_N, phase, text)
            except Exception as de:  # noqa: BLE001
                print(f"[control] flight recorder dump failed: {de}", flush=True)
            fault_buf.value = f"{code}|{text}".encode("utf-8", "replace")[:1023]
            status.value = ST_FAULT
        finally:
            # Release the ActiveControl object: while it is alive libfranka considers a control operation running,
            # and the next start_joint_position_control() fails with InvalidOperationException.
            ctrl = None
            gc.enable()
            gc.collect()
            if error:
                try:
                    robot.stop()
                except Exception:  # noqa: BLE001
                    pass
                try:
                    robot.automatic_error_recovery()
                except Exception:  # noqa: BLE001
                    pass
            else:
                status.value = ST_IDLE
            cmd.value = CMD_IDLE


class ControlClient:
    """Main-process handle to the control process."""

    def __init__(self, ip, impedance, v_max, a_max, j_max, wn, control_cpu=None):
        ctx = mp.get_context("spawn")
        self.K = list(impedance)
        self.target = ctx.RawArray("d", 14)
        self.t_target = ctx.RawValue("d", 0.0)
        self.state = ctx.RawArray("d", 7)
        self.cmd = ctx.RawValue("i", CMD_IDLE)
        self.status = ctx.RawValue("i", ST_IDLE)
        self.fault_q = ctx.RawArray("c", 1024)   # "code|message" of the last fault
        self.proc = ctx.Process(target=_control_process, daemon=True,
                                args=(ip, self.K, v_max, a_max, j_max, wn, self.target, self.t_target,
                                      self.state, self.cmd, self.status, self.fault_q, control_cpu))
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
                "joint_stiffness": self.ctrl.K,
                # internal libfranka joint impedance: damping is set by the robot itself, not configurable
                "joint_damping": [None] * 7,
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
    ctrl = ControlClient(args.ip, args.joint_impedance, args.v_max, args.a_max, args.j_max, args.filter_wn,
                         args.control_cpu)
    gripper = None if args.no_gripper else FciGripper(args.ip)
    srv = FrankaWebSocketServer(ctrl, gripper, cams, ver)
    try:
        async with websockets.serve(srv.handle, args.host, args.port, max_size=None, compression=None,
                                    ping_interval=20, ping_timeout=20):
            logger.info("Franka Robot Server on ws://%s:%d (FCI %s, joint-position control + internal impedance)",
                        args.host, args.port, args.ip)
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
    p.add_argument("--joint-impedance", type=float, nargs=7, default=DEFAULT_JOINT_IMPEDANCE,
                   help="libfranka internal joint impedance (Nm/rad), J1..J7")
    p.add_argument("--v-max", type=float, default=DEFAULT_V_MAX, help="reference filter max joint velocity (rad/s)")
    p.add_argument("--a-max", type=float, default=DEFAULT_A_MAX, help="reference filter max joint accel (rad/s^2)")
    p.add_argument("--j-max", type=float, default=DEFAULT_J_MAX, help="reference filter max joint jerk (rad/s^3)")
    p.add_argument("--filter-wn", type=float, default=DEFAULT_FILTER_WN, help="reference filter natural freq (rad/s)")
    p.add_argument("--agent-serial", default=AGENT_SERIAL, help="RealSense serial of the agent (external) camera")
    p.add_argument("--wrist-serial", default=WRIST_SERIAL, help="RealSense serial of the wrist camera")
    p.add_argument("--no-gripper", action="store_true")
    p.add_argument("--control-cpu", type=int, default=None,
                   help="pin the 1 kHz control process to this CPU (use the FCI NIC's IRQ CPU; see setup_rt.sh)")
    args = p.parse_args()
    try:
        asyncio.run(serve(args))
    except KeyboardInterrupt:
        logger.info("stopped by user")


if __name__ == "__main__":
    main()
