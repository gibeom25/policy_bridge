#!/usr/bin/env python3
"""Mock Robot Server & Mock Policy Server for testing robot_client.py end-to-end.

1. Mock Robot Server (ws://127.0.0.1:8000) — follows robot_server_format.json:
   - Sends robot_info once on connect
   - Streams state at 100 Hz and images (640x480 rgb8 base64) at 15 Hz
   - Receives joint_target (120 Hz), gripper, and stop
2. Mock Policy Server (ws://127.0.0.1:8001) — follows policy_output_format.json:
   - Receives MsgPack observation with numpy arrays
   - Replies with MsgPack action_chunk (16, 8) float32 + safety_info + timing_ms + server_timing
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import logging
import time
from typing import Any, Optional

import numpy as np
import websockets

from robot_client import pack_msg, unpack_msg

logger = logging.getLogger("mock_servers")


class MockRobotServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 8000) -> None:
        self.host = host
        self.port = port
        self.q = np.array([0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785], dtype=np.float64)
        self.gripper_pos = 0.0
        self.target_count = 0
        self.gripper_cmds: list[bool] = []
        self.stopped = False

        # Pre-encode a synthetic 640x480x3 rgb8 image (921600 bytes) in base64
        dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
        dummy_img[:, :, 0] = 64
        dummy_img[:, :, 1] = 128
        dummy_img[:, :, 2] = 192
        self._b64_img = base64.b64encode(dummy_img.tobytes()).decode("ascii")

    async def handle_client(self, ws: websockets.ServerConnection) -> None:
        self.stopped = False
        self.target_count = 0
        logger.info("[RobotServer] Client connected.")

        # 1. Send robot_info first
        robot_info = {
            "type": "robot_info",
            "robot_model": "FR3",
            "libfranka_version": "0.13.3",
            "joint_stiffness": [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
            "joint_damping": [49.0, 49.0, 49.0, 49.0, 31.6, 24.5, 14.1],
            "cameras": {"agent": "AGENT_CAM_01", "wrist": "WRIST_CAM_01"},
            "images_already_cropped": False,
        }
        await ws.send(json.dumps(robot_info))

        state_task = asyncio.create_task(self._stream_state(ws))
        image_task = asyncio.create_task(self._stream_images(ws))

        try:
            async for raw in ws:
                msg = json.loads(raw)
                mtype = msg.get("type")
                if mtype == "joint_target":
                    self.q = np.asarray(msg["q"], dtype=np.float64)
                    self.target_count += 1
                elif mtype == "gripper":
                    close = bool(msg["close"])
                    self.gripper_cmds.append(close)
                    self.gripper_pos = 1.0 if close else 0.0
                    logger.info("[RobotServer] Received gripper command: close=%s", close)
                elif mtype == "stop":
                    self.stopped = True
                    logger.info(
                        "[RobotServer] Received stop after %d joint_target messages.",
                        self.target_count,
                    )
                    break
        except websockets.ConnectionClosed:
            logger.info("[RobotServer] Connection closed.")
        finally:
            state_task.cancel()
            image_task.cancel()
            await asyncio.gather(state_task, image_task, return_exceptions=True)

    async def _stream_state(self, ws: websockets.ServerConnection) -> None:
        while not self.stopped:
            msg = {
                "type": "state",
                "joint_position": [float(x) for x in self.q],
                "gripper_position": float(self.gripper_pos),
            }
            await ws.send(json.dumps(msg))
            await asyncio.sleep(0.01)  # 100 Hz

    async def _stream_images(self, ws: websockets.ServerConnection) -> None:
        while not self.stopped:
            msg = {
                "type": "images",
                "agent": {
                    "encoding": "rgb8",
                    "width": 640,
                    "height": 480,
                    "data": self._b64_img,
                },
                "wrist": {
                    "encoding": "rgb8",
                    "width": 640,
                    "height": 480,
                    "data": self._b64_img,
                },
            }
            await ws.send(json.dumps(msg))
            await asyncio.sleep(1.0 / 15.0)  # 15 Hz


class MockPolicyServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 8001) -> None:
        self.host = host
        self.port = port

    async def handle_client(self, ws: websockets.ServerConnection) -> None:
        logger.info("[PolicyServer] Client connected.")
        prev_total_ms: Optional[float] = None
        last_req_time: Optional[float] = None
        step = 0

        try:
            async for raw in ws:
                t0 = time.perf_counter()
                if last_req_time is not None:
                    prev_total_ms = (t0 - last_req_time) * 1000.0
                last_req_time = t0

                obs = unpack_msg(raw)
                q_raw = obs.get("observation/joint_position", obs.get("joint_position"))
                q_curr = np.asarray(q_raw, dtype=np.float32)
                safety_req = obs.get("safety", {})
                mode = safety_req.get("mode", obs.get("mode", "correct"))

                # Build a smooth (16, 8) float32 action chunk
                chunk_plain = np.zeros((16, 8), dtype=np.float32)
                for i in range(16):
                    delta = 0.002 * float(i + 1) * (1.0 if step % 2 == 0 else -1.0)
                    chunk_plain[i, :7] = q_curr + delta
                    chunk_plain[i, 7] = 1.0 if step >= 1 else 0.0  # toggle gripper on step 1

                chunk_corrected = chunk_plain.copy()
                chunk_corrected[:, 0] += 0.001

                await asyncio.sleep(0.03)  # simulate 30 ms GPU inference
                infer_ms = (time.perf_counter() - t0) * 1000.0

                server_timing: dict[str, Any] = {"infer_ms": round(infer_ms, 2)}
                if prev_total_ms is not None:
                    server_timing["prev_total_ms"] = round(prev_total_ms, 2)

                resp: dict[str, Any] = {
                    "actions": chunk_corrected if mode == "correct" else chunk_plain,
                    "actions_plain": chunk_plain,
                    "actions_corrected": chunk_corrected,
                    "safety_info": {
                        "mode": mode,
                        "changed": True,
                        "max_dq_rad": 0.001,
                        "clear_before_m": -0.005,
                        "clear_after_m": 0.012,
                    },
                    "timing_ms": {"policy": 22.5, "correction": 7.5},
                    "server_timing": server_timing,
                }
                await ws.send(pack_msg(resp))
                step += 1
        except websockets.ConnectionClosed:
            logger.info("[PolicyServer] Connection closed.")


async def run_servers(robot_port: int, policy_port: int) -> None:
    robot_srv = MockRobotServer("127.0.0.1", robot_port)
    policy_srv = MockPolicyServer("127.0.0.1", policy_port)
    async with websockets.serve(
        robot_srv.handle_client, "127.0.0.1", robot_port, max_size=16 * 1024 * 1024
    ), websockets.serve(
        policy_srv.handle_client, "127.0.0.1", policy_port, max_size=16 * 1024 * 1024
    ):
        logger.info(
            "Mock Robot Server listening on ws://127.0.0.1:%d and Mock Policy Server on ws://127.0.0.1:%d",
            robot_port,
            policy_port,
        )
        await asyncio.Future()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(description="Run Mock Robot & Policy WebSocket Servers")
    parser.add_argument("--robot-port", type=int, default=8000)
    parser.add_argument("--policy-port", type=int, default=8001)
    args = parser.parse_args()
    try:
        asyncio.run(run_servers(args.robot_port, args.policy_port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
