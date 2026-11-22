#!/usr/bin/env python3
"""Robot Server (JSON/WS, robot_server.py) <-> CaCoFlow policy server (HTTP, cacoflow_client.py).

cacoflow_client.py is the collaborator's client, used unchanged: its run_trial() does the protocol
(/reset, /predict, /end_trial), the stale-observation checks and the action semantics
(q_target = q_measured_now + actions[k, :7], 30 Hz, synchronous inference). This file only
implements its RobotInterface on top of our Robot Server WebSocket.

    export CACOFLOW_API_TOKEN=...      (or let run_cacoflow.sh ask for it)
    python cacoflow_bridge.py --info-only --ssh-host 155.230.12.123 --ssh-user tanvir
    python cacoflow_bridge.py --ssh-host 155.230.12.123 --ssh-user tanvir \
        --instruction "put the blue mug in the bowl" --gripper-one-means closed --trial-id blue_bowl_p01

The policy server is reached through an SSH tunnel on a FREE local port (our Robot Server already
uses 127.0.0.1:8000), either the built-in one (--ssh-host) or your own `ssh -L` (--policy-url).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import threading
import time
from typing import Any, Optional

import numpy as np
import websockets

from cacoflow_client import InferenceClient, RobotInterface, run_trial
from robot_client2 import SSHTunnel, decode_rgb8_image, zoom_crop

logger = logging.getLogger("cacoflow_bridge")


class WsRobot(RobotInterface):
    """RobotInterface over the Robot Server WebSocket (asyncio loop in a background thread).

    Robot Server facts this relies on: it streams state at 100 Hz and both camera frames together at
    15 Hz (full 640x480 RGB, no capture timestamp -> the receive time is used as the capture time, so
    the real observation is slightly older than reported); it holds the last joint_target forever
    and smooths it with its jerk-limited reference filter (dq is ignored)."""

    def __init__(self, url: str, gripper_one_means: str, agent_zoom: float = 1.0,
                 max_state_age: float = 0.1, connect_timeout: float = 15.0) -> None:
        self.url = url
        self.one_is_closed = gripper_one_means == "closed"
        self.agent_zoom = agent_zoom
        self.max_state_age = max_state_age
        self._lock = threading.Lock()
        self._q: Optional[np.ndarray] = None
        self._g = 0.0                      # Robot Server convention: 0 = open, 1 = closed
        self._t_state = 0.0
        self._agent = self._wrist = None
        self._t_images = 0.0
        self._error: Optional[str] = None
        self._ready = threading.Event()
        self._last_close: Optional[bool] = None
        self._ws: Any = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self._recv_future = asyncio.run_coroutine_threadsafe(self._run(), self._loop)
        if not self._ready.wait(connect_timeout):
            raise RuntimeError(f"Robot Server at {url}: no robot_info/state/images within {connect_timeout} s")
        self._check()

    # --- WebSocket side (background thread) ---
    async def _run(self) -> None:
        try:
            async with websockets.connect(self.url, max_size=16 * 1024 * 1024,
                                          ping_interval=20, ping_timeout=20) as ws:
                self._ws = ws
                info = None
                async for raw in ws:
                    if len(raw) > 65536:
                        msg = await asyncio.to_thread(json.loads, raw)
                    else:
                        msg = json.loads(raw)
                    t = msg.get("type")
                    if t == "robot_info":
                        info = msg
                        logger.info("robot_info: model=%s cameras=%s", msg.get("robot_model"), msg.get("cameras"))
                    elif t == "state":
                        q = np.asarray(msg["joint_position"], dtype=np.float32)
                        with self._lock:
                            self._q, self._g, self._t_state = q, float(np.clip(msg["gripper_position"], 0, 1)), time.monotonic()
                    elif t == "images":
                        agent, wrist = await asyncio.to_thread(self._decode, msg, info)
                        with self._lock:
                            self._agent, self._wrist, self._t_images = agent, wrist, time.monotonic()
                    elif t == "error":
                        self._error = f"{msg.get('code')}: {msg.get('message')}"
                        logger.error("Robot Server error %s", self._error)
                        break
                    if info is not None and self._q is not None and self._agent is not None:
                        self._ready.set()
        except Exception as exc:  # noqa: BLE001
            if self._error is None:
                self._error = f"connection: {type(exc).__name__}: {exc}"
        finally:
            if self._error is None:
                self._error = "Robot Server closed the connection"
            self._ready.set()

    def _decode(self, msg: dict, info: Optional[dict]) -> tuple[np.ndarray, np.ndarray]:
        agent, wrist = decode_rgb8_image(msg["agent"]), decode_rgb8_image(msg["wrist"])
        if self.agent_zoom != 1.0 and not (info or {}).get("images_already_cropped", False):
            agent = zoom_crop(agent, self.agent_zoom)
        return agent, wrist

    def _send(self, msg: dict) -> None:
        if self._ws is None:
            raise RuntimeError("Robot Server not connected")
        asyncio.run_coroutine_threadsafe(self._ws.send(json.dumps(msg)), self._loop).result(timeout=1.0)

    def _check(self) -> None:
        if self._error is not None:
            raise RuntimeError(f"Robot Server: {self._error}")

    # --- RobotInterface (called from run_trial's thread) ---
    def observe(self):
        self._check()
        with self._lock:
            agent, wrist, q, g, t_img, t_state = self._agent, self._wrist, self._q, self._g, self._t_images, self._t_state
        now = time.monotonic()
        if now - t_state > self.max_state_age:
            raise RuntimeError(f"stale robot state ({(now - t_state) * 1e3:.0f} ms)")
        opening = g if self.one_is_closed else 1.0 - g
        captured = min(t_img, t_state)          # older of frames / state, robot PC monotonic clock
        stamp_ns = time.time_ns() - int((now - captured) * 1e9)
        return agent, wrist, q.copy(), float(opening), captured, stamp_ns

    def joints(self):
        self._check()
        with self._lock:
            q, t_state = self._q, self._t_state
        age = time.monotonic() - t_state
        if age > self.max_state_age:
            raise RuntimeError(f"stale robot state ({age * 1e3:.0f} ms)")
        return q.copy()

    def command(self, joint_target, gripper):
        self._check()
        q = np.asarray(joint_target, dtype=np.float64).reshape(7)
        if not np.isfinite(q).all():
            raise RuntimeError("non-finite joint target")
        self._send({"type": "joint_target", "q": q.tolist(), "dq": [0.0] * 7})
        close = (gripper >= 0.5) == self.one_is_closed
        if close != self._last_close:
            self._send({"type": "gripper", "close": bool(close)})
            self._last_close = close
            logger.info("gripper close=%s", close)

    def hold(self):
        """Hold where the robot is now. Never raises (called from run_trial's finally)."""
        try:
            if self._error is None and self._q is not None:
                self._send({"type": "joint_target", "q": self._q.astype(float).tolist(), "dq": [0.0] * 7})
        except Exception as exc:  # noqa: BLE001
            logger.warning("hold failed: %s", exc)

    def close(self) -> None:
        try:
            if self._error is None:
                self._send({"type": "stop"})
        except Exception as exc:  # noqa: BLE001
            logger.warning("stop failed: %s", exc)
        if self._ws is not None:
            asyncio.run_coroutine_threadsafe(self._ws.close(), self._loop)
        try:
            self._recv_future.result(timeout=3.0)
        except Exception:  # noqa: BLE001
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)


def ask_outcome() -> tuple[Optional[bool], Optional[bool], str]:
    def yn(prompt: str) -> Optional[bool]:
        a = input(prompt).strip().lower()
        return True if a in ("y", "yes") else False if a in ("n", "no") else None
    success = yn("Trial success? [y/n, Enter = undecided]: ")
    wrong = yn("Wrong object (partner instruction's object)? [y/n, Enter = n/a]: ")
    notes = input("Notes (optional): ").strip()
    return success, wrong, notes


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot-url", default="ws://127.0.0.1:8000")
    p.add_argument("--policy-url", default=None,
                   help="CaCoFlow server through your own tunnel, e.g. http://127.0.0.1:8100 (not 8000: Robot Server)")
    p.add_argument("--info-only", action="store_true", help="print the server's /info and exit (no robot)")
    p.add_argument("--instruction", help="task sentence; must be one of /info's tasks")
    p.add_argument("--gripper-one-means", choices=["closed", "open"],
                   help="REQUIRED for trials: what 1 means in the recordings (gripper_opening and action column 7). "
                        "The handoff does not state it; confirm with the collaborator")
    p.add_argument("--trial-id", default=None, help="unique id; the server logs the trial when started with --log_dir")
    p.add_argument("--placement", default=None)
    p.add_argument("--appearance", choices=["seen", "unseen"], default="seen")
    p.add_argument("--max-steps", type=int, default=600, help="30 Hz control steps (600 = 20 s)")
    p.add_argument("--agent-zoom", type=float, default=1.0,
                   help="center zoom of the agent camera as in data collection (handoff mentions 1.2; confirm)")
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument("--max-observation-age", type=float, default=0.5)
    g = p.add_argument_group("built-in SSH tunnel (alternative to --policy-url)")
    g.add_argument("--ssh-host", default=None, help="e.g. 155.230.12.123")
    g.add_argument("--ssh-port", type=int, default=22)
    g.add_argument("--ssh-user", default="tanvir")
    g.add_argument("--ssh-key", default=None)
    g.add_argument("--remote-policy-port", type=int, default=8000)
    args = p.parse_args()

    token = os.environ.get("CACOFLOW_API_TOKEN", "")
    if not token:
        p.error("set CACOFLOW_API_TOKEN (run_cacoflow.sh asks for it)")
    if not args.info_only and (not args.instruction or not args.gripper_one_means):
        p.error("--instruction and --gripper-one-means are required for a trial")
    if not args.ssh_host and not args.policy_url:
        p.error("give --ssh-host (built-in tunnel) or --policy-url")

    tunnel = robot = None
    try:
        policy_url = args.policy_url
        if args.ssh_host:
            tunnel = SSHTunnel(args.ssh_host, args.ssh_port, args.ssh_user, ssh_key=args.ssh_key,
                               remote_host="127.0.0.1", remote_port=args.remote_policy_port)
            policy_url = f"http://127.0.0.1:{tunnel.start()}"
        client = InferenceClient(policy_url, token, args.timeout, args.max_observation_age)
        info = client.info
        print(json.dumps(info, indent=2))
        if args.info_only:
            return
        if info.get("mock"):
            raise SystemExit("Server is in mock mode: never execute its actions on the robot.")
        if list(info.get("action_shape", [])) != [10, 8]:
            raise SystemExit(f"Unexpected action_shape {info.get('action_shape')} (handoff says [10, 8])")
        if args.instruction not in info.get("tasks", []):
            raise SystemExit(f"Instruction not in the server's tasks: {info.get('tasks')}")

        robot = WsRobot(args.robot_url, args.gripper_one_means, args.agent_zoom)
        t_steps = []

        def on_step(step, k, row, target, gripper):
            t_steps.append(time.monotonic())

        result = run_trial(client, robot, args.instruction, trial_id=args.trial_id, placement=args.placement,
                           appearance=args.appearance, max_steps=args.max_steps, on_step=on_step)
        robot.close()
        robot = None
        rt = [t["roundtrip_ms"] for t in result["timing"]]
        logger.info("%d steps in %d chunks, clipped steps %d, round trip median %.0f ms (max %.0f)",
                    result["steps"], result["chunks"], result["clipped_steps"],
                    float(np.median(rt)) if rt else float("nan"), max(rt) if rt else float("nan"))
        success, wrong, notes = ask_outcome()
        end = client.end_trial(success=success, wrong_object=wrong, notes=notes)
        logger.info("end_trial: %s", end.get("trial_dir", end))
    finally:
        if robot is not None:
            robot.hold()
            robot.close()
        if tunnel is not None:
            tunnel.close()


if __name__ == "__main__":
    main()
