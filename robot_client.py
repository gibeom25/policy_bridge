#!/usr/bin/env python3
"""Robot Client Bridge: Robot Server (JSON/WS) <-> Policy Server (MsgPack/WS).

Implements:
  1. robot_server_format.json (WebSocket JSON text frames)
     - Receives: robot_info (once), state (>=100 Hz), images (>=15 Hz, raw rgb8 base64), error
     - Sends:    joint_target (120 Hz while moving), gripper (on change only), stop (once at end)
  2. policy_output_format.json (WebSocket MsgPack binary frames, optionally via SSH tunnel)
     - Sends:    latest robot state & decoded RGB images (with __ndarray__ msgpack encoding)
     - Receives: action_chunk (16, 8) float32 [cols 0..6 = joint rad, col 7 = gripper 0..1],
                 safety_info, timing_ms, server_timing
"""
from __future__ import annotations

import PIL
import argparse
import asyncio
import base64
import json
import logging
import math
import select
import socket
import socketserver
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import msgpack
import numpy as np
import websockets

logger = logging.getLogger("robot_client")


# ============================================================================
# 1. MsgPack <-> NumPy encoding (matches policy_output_format.json)
#    {"__ndarray__": true, "data": "<raw bytes>", "dtype": "<f4", "shape": [16, 8]}
# ============================================================================

def encode_ndarray(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }
    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": obj.item(),
            b"dtype": obj.dtype.str,
        }
    return obj


def decode_ndarray(obj: Any) -> Any:
    if isinstance(obj, dict):
        if obj.get(b"__ndarray__") is True:
            dtype = np.dtype(obj[b"dtype"])
            shape = tuple(obj[b"shape"])
            return np.frombuffer(obj[b"data"], dtype=dtype).reshape(shape).copy()
        if obj.get(b"__npgeneric__") is True:
            return np.dtype(obj[b"dtype"]).type(obj[b"data"])
        if obj.get("__ndarray__") is True:
            dtype = np.dtype(obj["dtype"])
            shape = tuple(obj["shape"])
            return np.frombuffer(obj["data"], dtype=dtype).reshape(shape).copy()
    return obj


def pack_msg(data: dict[str, Any]) -> bytes:
    return msgpack.packb(data, default=encode_ndarray, use_bin_type=True)


def unpack_msg(raw: bytes) -> dict[str, Any]:
    return msgpack.unpackb(raw, object_hook=decode_ndarray, raw=False)


# ============================================================================
# 2. Optional SSH Local Port Forwarding Tunnel (for remote GPU Policy Server)
# ============================================================================

class _ForwardHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        try:
            chan = self.server.ssh_transport.open_channel(  # type: ignore[attr-defined]
                "direct-tcpip",
                (self.server.remote_host, self.server.remote_port),  # type: ignore[attr-defined]
                self.request.getpeername(),
            )
        except Exception as exc:
            logger.error("SSH tunnel channel open failed: %s", exc)
            return
        if chan is None:
            return
        try:
            while True:
                r, _, _ = select.select([self.request, chan], [], [], 1.0)
                if self.request in r:
                    data = self.request.recv(16384)
                    if not data:
                        break
                    chan.sendall(data)
                if chan in r:
                    data = chan.recv(16384)
                    if not data:
                        break
                    self.request.sendall(data)
        finally:
            chan.close()
            self.request.close()


class _ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = True


class SSHTunnel:
    """Forwards a local port to (remote_host, remote_port) over SSH."""

    def __init__(
        self,
        ssh_host: str,
        ssh_port: int,
        ssh_user: str,
        ssh_password: Optional[str] = None,
        ssh_key: Optional[str] = None,
        remote_host: str = "127.0.0.1",
        remote_port: int = 8001,
    ) -> None:
        self.ssh_host = ssh_host
        self.ssh_port = ssh_port
        self.ssh_user = ssh_user
        self.ssh_password = ssh_password
        self.ssh_key = ssh_key
        self.remote_host = remote_host
        self.remote_port = remote_port
        self.client: Any = None
        self.server: Optional[_ThreadedTCPServer] = None
        self.local_port: int = 0

    def start(self) -> int:
        import getpass
        import paramiko

        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        logger.info(
            "Connecting SSH tunnel to %s@%s:%d ...",
            self.ssh_user,
            self.ssh_host,
            self.ssh_port,
        )
        try:
            self.client.connect(
                hostname=self.ssh_host,
                port=self.ssh_port,
                username=self.ssh_user,
                password=self.ssh_password,
                key_filename=self.ssh_key,
                timeout=1000.0,
            )
        except paramiko.AuthenticationException:
            if self.ssh_password is not None:
                raise
            pw = getpass.getpass(f"{self.ssh_user}@{self.ssh_host}'s password: ")
            self.client.connect(
                hostname=self.ssh_host,
                port=self.ssh_port,
                username=self.ssh_user,
                password=pw,
                look_for_keys=False,
                allow_agent=False,
                timeout=100.0,
            )
        self.server = _ThreadedTCPServer(("127.0.0.1", 0), _ForwardHandler)
        self.server.ssh_transport = self.client.get_transport()  # type: ignore[attr-defined]
        self.server.remote_host = self.remote_host  # type: ignore[attr-defined]
        self.server.remote_port = self.remote_port  # type: ignore[attr-defined]
        self.local_port = self.server.server_address[1]
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        logger.info(
            "SSH tunnel established: 127.0.0.1:%d -> %s:%d (via %s:%d)",
            self.local_port,
            self.remote_host,
            self.remote_port,
            self.ssh_host,
            self.ssh_port,
        )
        return self.local_port

    def close(self) -> None:
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
        if self.client is not None:
            self.client.close()
            self.client = None


# ============================================================================
# 3. Trajectory & Formatting Helpers (robot_server_format.json rules)
# ============================================================================

def min_jerk(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def min_jerk_dot(s: float, duration: float) -> float:
    s = min(max(s, 0.0), 1.0)
    if duration <= 0.0 or s >= 1.0:
        return 0.0
    return (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / duration


def ensure_finite_list(values: np.ndarray | list[float], expected_len: int = 7) -> list[float]:
    """Ensure finite numbers only (NaN and Infinity are forbidden by robot_server_format.json)."""
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.size != expected_len:
        raise ValueError(f"Expected {expected_len} joint values, got {arr.size}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"Non-finite values detected in joint array: {arr}")
    return [float(x) for x in arr]


def decode_rgb8_image(img_dict: dict[str, Any]) -> np.ndarray:
    """Decode base64 raw rgb8 image (640x480x3 = 921600 bytes) into uint8 ndarray (H, W, 3)."""
    enc = img_dict.get("encoding")
    if enc != "rgb8":
        raise ValueError(f"Expected 'rgb8' encoding, got {enc!r}")
    w = int(img_dict["width"])
    h = int(img_dict["height"])
    raw = base64.b64decode(img_dict["data"])
    expected_bytes = w * h * 3
    if len(raw) != expected_bytes:
        raise ValueError(f"Expected {expected_bytes} image bytes, got {len(raw)}")
    return np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))


# ============================================================================
# 4. Robot State Buffer & Bridge Client
# ============================================================================

@dataclass
class RobotSessionState:
    robot_info: Optional[dict[str, Any]] = None
    joint_position: Optional[np.ndarray] = None  # (7,) float32, rad
    gripper_position: float = 0.0                # 0..1 (0=open, 1=closed)
    state_timestamp: float = 0.0
    state_count: int = 0

    agent_image: Optional[np.ndarray] = None     # (480, 640, 3) uint8
    wrist_image: Optional[np.ndarray] = None     # (480, 640, 3) uint8
    image_timestamp: float = 0.0
    image_count: int = 0

    error: Optional[dict[str, Any]] = None
    ready_event: asyncio.Event = field(default_factory=asyncio.Event)
    error_event: asyncio.Event = field(default_factory=asyncio.Event)

    def check_ready(self) -> None:
        if (
            self.robot_info is not None
            and self.joint_position is not None
            and self.agent_image is not None
            and self.wrist_image is not None
        ):
            self.ready_event.set()


class RobotPolicyBridge:
    """Bridges Robot Server (JSON/WS) and Policy Server (MsgPack/WS)."""

    # Timing constants from policy_output_format.json & robot_server_format.json
    POLICY_RATE_HZ: float = 15.0
    SECONDS_PER_ACTION: float = 1.0 / 15.0        # 0.0667 s
    ACTIONS_PER_DECISION: int = 8                 # 8 actions executed out of 16
    ROBOT_TARGET_RATE_HZ: float = 120.0           # 120 Hz joint_target rate while moving
    SUBSTEPS_PER_ACTION: int = 8                  # 120 Hz / 15 Hz = 8 ticks per policy action
    GRIPPER_THRESHOLD: float = 0.5                # column 7 threshold: <0.5 open, >=0.5 close

    def __init__(
        self,
        robot_url: str,
        policy_url: str,
        mode: str = "correct",
        prompt: str = "",
        start_pose: Optional[list[float]] = None,
        start_move_duration: float = 3.0,
        max_decisions: int = 0,
        crop_size: Optional[int] = None,
    ) -> None:
        self.robot_url = robot_url
        self.policy_url = policy_url
        self.mode = mode
        self.prompt = prompt
        self.start_pose = np.asarray(start_pose, dtype=np.float64) if start_pose is not None else None
        self.start_move_duration = start_move_duration
        self.max_decisions = max_decisions
        self.crop_size = crop_size

        self.session = RobotSessionState()
        self.last_gripper_close: Optional[bool] = None
        self.last_commanded_q: Optional[np.ndarray] = None
        self._stop_sent: bool = False

    # ------------------------------------------------------------------
    # Robot Server message senders (JSON text frames, no replies expected)
    # ------------------------------------------------------------------
    async def send_joint_target(
        self,
        robot_ws: websockets.ClientConnection,
        q: np.ndarray,
        dq: np.ndarray,
    ) -> None:
        msg = {
            "type": "joint_target",
            "q": ensure_finite_list(q, 7),
            "dq": ensure_finite_list(dq, 7),
        }
        await robot_ws.send(json.dumps(msg))
        self.last_commanded_q = np.asarray(q, dtype=np.float64).copy()

    async def send_gripper_if_changed(
        self,
        robot_ws: websockets.ClientConnection,
        close: bool,
    ) -> None:
        """Send gripper command ONLY when the command changes (per robot_server_format.json)."""
        if self.last_gripper_close is not None and self.last_gripper_close == close:
            return
        msg = {"type": "gripper", "close": bool(close)}
        await robot_ws.send(json.dumps(msg))
        self.last_gripper_close = bool(close)
        logger.info("Sent gripper command: close=%s", close)

    async def send_stop(self, robot_ws: websockets.ClientConnection) -> None:
        """Send stop once at the end of a trial."""
        if self._stop_sent:
            return
        self._stop_sent = True
        try:
            await robot_ws.send(json.dumps({"type": "stop"}))
            logger.info("Sent stop message to Robot Server.")
        except Exception as exc:
            logger.warning("Failed to send stop message: %s", exc)

    # ------------------------------------------------------------------
    # Robot Server receiver loop (runs concurrently without blocking)
    # ------------------------------------------------------------------
    async def _robot_recv_loop(self, robot_ws: websockets.ClientConnection) -> None:
        async for raw_frame in robot_ws:
            if not isinstance(raw_frame, str):
                logger.warning("Ignored non-text frame from Robot Server")
                continue
            msg = json.loads(raw_frame)
            msg_type = msg.get("type")

            if msg_type == "robot_info":
                self.session.robot_info = msg
                logger.info(
                    "Received robot_info: model=%s, libfranka=%s, cameras=%s, cropped=%s",
                    msg.get("robot_model"),
                    msg.get("libfranka_version"),
                    msg.get("cameras"),
                    msg.get("images_already_cropped"),
                )
                self.session.check_ready()

            elif msg_type == "state":
                q = np.asarray(msg["joint_position"], dtype=np.float32)
                g = float(np.clip(msg["gripper_position"], 0.0, 1.0))
                self.session.joint_position = q
                self.session.gripper_position = g
                self.session.state_timestamp = time.monotonic()
                self.session.state_count += 1
                self.session.check_ready()

            elif msg_type == "images":
                agent_img = decode_rgb8_image(msg["agent"])
                wrist_img = decode_rgb8_image(msg["wrist"])
                if (
                    self.crop_size is not None
                    and self.session.robot_info is not None
                    and not self.session.robot_info.get("images_already_cropped", False)
                ):
                    agent_img = self._center_crop(agent_img, self.crop_size)
                    wrist_img = self._center_crop(wrist_img, self.crop_size)
                self.session.agent_image = agent_img
                self.session.wrist_image = wrist_img
                self.session.image_timestamp = time.monotonic()
                self.session.image_count += 1
                self.session.check_ready()

            elif msg_type == "error":
                self.session.error = msg
                logger.error(
                    "Robot Server reported fault! code=%s, message=%s",
                    msg.get("code"),
                    msg.get("message"),
                )
                self.session.error_event.set()
                break

            else:
                logger.warning("Unknown message type from Robot Server: %r", msg_type)

    @staticmethod
    def _center_crop(img: np.ndarray, size: int) -> np.ndarray:
        h, w, _ = img.shape
        if size >= h or size >= w:
            return img
        top = (h - size) // 2
        left = (w - size) // 2
        return np.ascontiguousarray(img[top : top + size, left : left + size])

    # ------------------------------------------------------------------
    # Motion routines: 120 Hz start-pose move & 120 Hz chunk execution
    # ------------------------------------------------------------------
    async def move_to_start_pose(
        self,
        robot_ws: websockets.ClientConnection,
        q_goal: np.ndarray,
        duration: float,
    ) -> None:
        """Slow min-jerk move to the start pose at 120 Hz before running the policy."""
        assert self.session.joint_position is not None
        q_start = self.session.joint_position.astype(np.float64).copy()
        q_goal = np.asarray(q_goal, dtype=np.float64)
        max_delta = float(np.max(np.abs(q_goal - q_start)))
        if max_delta < 1e-4:
            logger.info("Already at start pose (max_delta=%.6f rad).", max_delta)
            self.last_commanded_q = q_start.copy()
            return

        logger.info(
            "Moving slowly to start pose over %.2f s (max delta %.3f rad) at 120 Hz...",
            duration,
            max_delta,
        )
        dt = 1.0 / self.ROBOT_TARGET_RATE_HZ
        steps = max(1, int(round(duration * self.ROBOT_TARGET_RATE_HZ)))
        t0 = time.perf_counter()

        for step_idx in range(1, steps + 1):
            if self.session.error_event.is_set():
                raise RuntimeError(f"Robot error during start move: {self.session.error}")
            s = step_idx / steps
            alpha = min_jerk(s)
            dalpha = min_jerk_dot(s, duration)
            q_cmd = q_start + alpha * (q_goal - q_start)
            dq_cmd = dalpha * (q_goal - q_start)
            await self.send_joint_target(robot_ws, q_cmd, dq_cmd)

            t_target = t0 + step_idx * dt
            sleep_s = t_target - time.perf_counter()
            if sleep_s > 0:
                await asyncio.sleep(sleep_s)

        logger.info("Reached start pose.")

    async def execute_action_chunk(
        self,
        robot_ws: websockets.ClientConnection,
        actions: np.ndarray,
    ) -> None:
        """Execute the first 8 actions of a (16, 8) chunk at 15 Hz, upsampled to 120 Hz joint_target."""
        if actions.ndim != 2 or actions.shape[1] != 8:
            raise ValueError(f"Expected actions shape (N, 8), got {actions.shape}")

        num_to_exec = min(self.ACTIONS_PER_DECISION, actions.shape[0])
        dt_robot = 1.0 / self.ROBOT_TARGET_RATE_HZ  # 1/120 s = 8.333 ms
        dt_action = self.SECONDS_PER_ACTION         # 1/15 s  = 66.667 ms

        if self.last_commanded_q is not None:
            q_prev = self.last_commanded_q.copy()
        else:
            assert self.session.joint_position is not None
            q_prev = self.session.joint_position.astype(np.float64).copy()

        t0 = time.perf_counter()
        tick = 0

        for act_idx in range(num_to_exec):
            if self.session.error_event.is_set():
                raise RuntimeError(f"Robot error during chunk execution: {self.session.error}")

            row = actions[act_idx]
            q_next = np.asarray(row[:7], dtype=np.float64)
            gripper_val = float(row[7])

            # Threshold gripper at 0.5 and send ONLY if changed
            close_cmd = gripper_val >= self.GRIPPER_THRESHOLD
            await self.send_gripper_if_changed(robot_ws, close_cmd)

            # Constant feed-forward velocity across the 66.7ms action step
            dq_step = (q_next - q_prev) / dt_action

            # Upsample 15 Hz action step into 8 substeps at 120 Hz
            for sub in range(1, self.SUBSTEPS_PER_ACTION + 1):
                if self.session.error_event.is_set():
                    raise RuntimeError(f"Robot error during chunk execution: {self.session.error}")
                frac = sub / self.SUBSTEPS_PER_ACTION
                q_interp = q_prev + frac * (q_next - q_prev)
                dq_cmd = dq_step if act_idx < num_to_exec - 1 or sub < self.SUBSTEPS_PER_ACTION else np.zeros(7)
                await self.send_joint_target(robot_ws, q_interp, dq_cmd)

                tick += 1
                t_target = t0 + tick * dt_robot
                sleep_s = t_target - time.perf_counter()
                if sleep_s > 0:
                    await asyncio.sleep(sleep_s)

            q_prev = q_next

    # ------------------------------------------------------------------
    # Policy Server request/response (matches realrobot/serve.py & policy_output_format.json)
    # ------------------------------------------------------------------
    @staticmethod
    def _resize_bilinear(img: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
        """Pure-NumPy bilinear resize for HxWx3 uint8 images."""
        in_h, in_w = img.shape[:2]
        if (in_h, in_w) == (out_h, out_w):
            return img
        y = np.linspace(0, in_h - 1, out_h, dtype=np.float32)
        x = np.linspace(0, in_w - 1, out_w, dtype=np.float32)
        y0 = np.floor(y).astype(np.int32)
        x0 = np.floor(x).astype(np.int32)
        y1 = np.minimum(y0 + 1, in_h - 1)
        x1 = np.minimum(x0 + 1, in_w - 1)
        wy = (y - y0)[:, None, None]
        wx = (x - x0)[None, :, None]
        top = (1.0 - wx) * img[y0[:, None], x0[None, :]] + wx * img[y0[:, None], x1[None, :]]
        bot = (1.0 - wx) * img[y1[:, None], x0[None, :]] + wx * img[y1[:, None], x1[None, :]]
        return np.clip(np.round((1.0 - wy) * top + wy * bot), 0, 255).astype(np.uint8)

    @classmethod
    def _apply_crop(cls, img: np.ndarray, zoom: float = 1.0, x: int = 0, y: int = 0) -> np.ndarray:
        if zoom == 1.0 and x == 0 and y == 0:
            return img
        H, W = img.shape[:2]
        h, w = round(H / zoom), round(W / zoom)
        t = min(max((H - h) // 2 + y, 0), H - h)
        l = min(max((W - w) // 2 + x, 0), W - w)
        return cls._resize_bilinear(img[t : t + h, l : l + w], H, W)

    @classmethod
    def _resize_pad(cls, img: np.ndarray, size: int = 224) -> np.ndarray:
        """Keep aspect ratio and pad to size x size (matches convert_to_lerobot.resize_pad)."""
        in_h, in_w = img.shape[:2]
        s = size / max(in_w, in_h)
        new_w = max(1, round(in_w * s))
        new_h = max(1, round(in_h * s))
        resized = cls._resize_bilinear(img, new_h, new_w)
        out = np.zeros((size, size, 3), dtype=np.uint8)
        top = (size - new_h) // 2
        left = (size - new_w) // 2
        out[top : top + new_h, left : left + new_w] = resized
        return out

    def build_policy_request(self) -> dict[str, Any]:
        """Build the observation dictionary sent to realrobot/serve.py (OpenPI DROID format)."""
        assert self.session.joint_position is not None
        assert self.session.agent_image is not None
        assert self.session.wrist_image is not None

        q = self.session.joint_position.astype(np.float32)
        g = float(np.clip(self.session.gripper_position, 0.0, 1.0))
        q_prev = (
            self.last_commanded_q.astype(np.float32)
            if self.last_commanded_q is not None
            else q
        )

        agent_img = self.session.agent_image
        wrist_img = self.session.wrist_image
        already_cropped = bool(
            self.session.robot_info and self.session.robot_info.get("images_already_cropped", False)
        )
        if not already_cropped:
            agent_img = self._apply_crop(agent_img, zoom=1.2, x=0, y=0)
            wrist_img = self._apply_crop(wrist_img, zoom=1.0, x=31, y=0)

        req: dict[str, Any] = {
            "observation/exterior_image_1_left": self._resize_pad(agent_img, 224),
            "observation/wrist_image_left": self._resize_pad(wrist_img, 224),
            "observation/joint_position": q,
            "observation/gripper_position": np.asarray([g], dtype=np.float32),
            "prompt": self.prompt.strip().lower(),
            "joint_position_prev": q_prev,
            "safety": {
                "mode": self.mode,
                "boxes": [],
                "grasped": bool(g > self.GRIPPER_THRESHOLD),
            },
        }
        return req

    async def request_policy_chunk(
        self,
        policy_ws: websockets.ClientConnection,
    ) -> dict[str, Any]:
        """Send observation to Policy Server and unpack the msgpack response."""
        req = self.build_policy_request()
        await policy_ws.send(pack_msg(req))

        while True:
            raw_reply = await policy_ws.recv()
            # Per policy_output_format.json:
            # "a text frame holding a traceback, then the connection closes with code 1011; a text frame is always an error"
            if isinstance(raw_reply, str):
                raise RuntimeError(f"Policy Server returned error traceback:\n{raw_reply}")

            resp = unpack_msg(raw_reply)
            if "actions" in resp:
                return resp
            # OpenPI WebsocketPolicyServer sends an initial server metadata dict (often `{}`) on connect
            logger.info("Received Policy Server handshake metadata: %s", resp)

    # ------------------------------------------------------------------
    # Main Trial Session Loop
    # ------------------------------------------------------------------
    async def run_trial(self) -> None:
        logger.info("Connecting to Robot Server at %s ...", self.robot_url)
        logger.info("Connecting to Policy Server at %s ...", self.policy_url)

        async with websockets.connect(
            self.robot_url,
            max_size=16 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=20,
        ) as robot_ws, websockets.connect(
            self.policy_url,
            max_size=16 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=20,
        ) as policy_ws:
            recv_task = asyncio.create_task(self._robot_recv_loop(robot_ws))
            try:
                logger.info("Waiting for initial robot_info, state, and camera images...")
                await asyncio.wait_for(self.session.ready_event.wait(), timeout=15.0)
                logger.info(
                    "Robot stream ready! Initial joints (rad): %s, gripper: %.3f",
                    np.round(self.session.joint_position, 4),  # type: ignore[arg-type]
                    self.session.gripper_position,
                )

                # Step 1: Optional slow move to start pose before policy execution
                if self.start_pose is not None:
                    await self.move_to_start_pose(
                        robot_ws,
                        self.start_pose,
                        self.start_move_duration,
                    )

                # Step 2: Policy inference & execution loop
                decision_idx = 0
                while True:
                    if self.session.error_event.is_set():
                        raise RuntimeError(f"Stopping trial due to robot error: {self.session.error}")
                    if 0 < self.max_decisions <= decision_idx:
                        logger.info("Reached max_decisions=%d. Finishing trial.", self.max_decisions)
                        break

                    # Targets pause while the policy runs; Robot Server holds the last target
                    t_infer_start = time.perf_counter()
                    resp = await self.request_policy_chunk(policy_ws)
                    infer_wall_ms = (time.perf_counter() - t_infer_start) * 1000.0

                    actions = np.asarray(resp["actions"], dtype=np.float32)
                    safety_info = resp.get("safety_info", {})
                    timing_ms = resp.get("timing_ms", {})
                    server_timing = resp.get("server_timing", {})

                    logger.info(
                        "[Decision %d] actions=%s | mode=%s changed=%s max_dq=%s | "
                        "policy=%.1fms corr=%.1fms infer=%.1fms (wall=%.1fms)",
                        decision_idx,
                        actions.shape,
                        safety_info.get("mode"),
                        safety_info.get("changed"),
                        safety_info.get("max_dq_rad"),
                        timing_ms.get("policy", 0.0),
                        timing_ms.get("correction", 0.0),
                        server_timing.get("infer_ms", 0.0),
                        infer_wall_ms,
                    )

                    await self.execute_action_chunk(robot_ws, actions)
                    decision_idx += 1

            except asyncio.CancelledError:
                logger.info("Trial cancelled.")
                raise
            finally:
                await self.send_stop(robot_ws)
                recv_task.cancel()
                try:
                    await recv_task
                except asyncio.CancelledError:
                    pass


# ============================================================================
# 5. CLI Entrypoint
# ============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Robot Client Bridge (Robot Server JSON/WS <-> Policy Server MsgPack/WS)"
    )
    parser.add_argument(
        "--robot-url",
        default="ws://127.0.0.1:8000",
        help="Robot Server WebSocket URL (default: ws://127.0.0.1:8000)",
    )
    parser.add_argument(
        "--policy-url",
        default="ws://127.0.0.1:8001/",
        help="Policy Server WebSocket URL (default: ws://127.0.0.1:8001/)",
    )
    parser.add_argument(
        "--mode",
        choices=["plain", "shadow", "correct"],
        default="correct",
        help="Safety correction mode for the Policy Server (default: correct)",
    )
    parser.add_argument(
        "--prompt",
        default="",
        help="Optional language task prompt sent to the Policy Server",
    )
    parser.add_argument(
        "--start-pose",
        type=float,
        nargs=7,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="Optional 7 joint angles (rad) to slowly move to before running the policy",
    )
    parser.add_argument(
        "--start-move-duration",
        type=float,
        default=3.0,
        help="Duration (s) for the slow move to --start-pose (default: 3.0)",
    )
    parser.add_argument(
        "--max-decisions",
        type=int,
        default=0,
        help="Stop automatically after N policy decisions (0 = run until Ctrl+C)",
    )
    parser.add_argument(
        "--crop-size",
        type=int,
        default=None,
        help="Optional center-crop size if robot_info.images_already_cropped is false",
    )

    # Optional built-in SSH tunnel arguments for remote GPU servers
    ssh_group = parser.add_argument_group("SSH Tunnel Options (optional)")
    ssh_group.add_argument(
        "--ssh-host",
        default=None,
        help="SSH server hostname/IP to tunnel Policy Server connection through",
    )
    ssh_group.add_argument(
        "--ssh-port",
        type=int,
        default=22,
        help="SSH server port (default: 22, e.g. 17970)",
    )
    ssh_group.add_argument(
        "--ssh-user",
        default="gibeom",
        help="SSH username (default: gibeom)",
    )
    ssh_group.add_argument(
        "--ssh-password",
        default=None,
        help="SSH password (if not using SSH key)",
    )
    ssh_group.add_argument(
        "--ssh-key",
        default=None,
        help="Path to SSH private key file",
    )
    ssh_group.add_argument(
        "--remote-policy-host",
        default="127.0.0.1",
        help="Policy host as seen from the SSH server (default: 127.0.0.1)",
    )
    ssh_group.add_argument(
        "--remote-policy-port",
        type=int,
        default=8001,
        help="Policy port on the remote SSH server (default: 8001)",
    )
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    args = parse_args()

    tunnel: Optional[SSHTunnel] = None
    policy_url = args.policy_url

    try:
        if args.ssh_host:
            tunnel = SSHTunnel(
                ssh_host=args.ssh_host,
                ssh_port=args.ssh_port,
                ssh_user=args.ssh_user,
                ssh_password=args.ssh_password,
                ssh_key=args.ssh_key,
                remote_host=args.remote_policy_host,
                remote_port=args.remote_policy_port,
            )
            local_port = tunnel.start()
            policy_url = f"ws://127.0.0.1:{local_port}/"

        bridge = RobotPolicyBridge(
            robot_url=args.robot_url,
            policy_url=policy_url,
            mode=args.mode,
            prompt=args.prompt,
            start_pose=args.start_pose,
            start_move_duration=args.start_move_duration,
            max_decisions=args.max_decisions,
            crop_size=args.crop_size,
        )
        asyncio.run(bridge.run_trial())
    except KeyboardInterrupt:
        logger.info("Interrupted by user (Ctrl+C).")
    finally:
        if tunnel is not None:
            tunnel.close()


if __name__ == "__main__":
    main()
