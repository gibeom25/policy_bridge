"""CaCoFlow real-robot client (lab PC). Needs only numpy and Pillow (h5py for --replay).

Adapted from the lab's earlier real_robot_client_example.py. It adds what CaCoFlow needs:
the measured robot state in every request, the conversion of returned action rows into joint
targets, a 30 Hz trial loop, and trial bookkeeping (/reset with trial metadata, /end_trial).

The robot itself is behind the small RobotInterface below: implement it with the lab's robot SDK
and camera code (the same software, crop/zoom and RGB order used to record the dataset).
Nothing here talks to hardware. ReplayRobot plays a recorded HDF5 episode instead of a robot, to
test the whole client -> server -> command path without moving anything.

    # 1) GPU server: python real_exp/serve_cacoflow.py --checkpoint ... --log_dir /data1/robot/real_trials
    # 2) lab PC:     ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8000:127.0.0.1:8000 USER@SERVER
    # 3) lab PC:     export CACOFLOW_API_TOKEN=...   (same token as the server)
    python cacoflow_client.py --info-only
    python cacoflow_client.py --replay /path/put_the_blue_mug_in_the_bowl_normal.hdf5 --episode episode_000
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import time
import uuid
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

import numpy as np
from PIL import Image

MAX_RESPONSE_BYTES = 4 * 1024 * 1024
CONTROL_HZ = 30.0

# Safety limits applied to every command on the client (verify for your arm; these are the
# intersection of the Franka Panda and FR3 joint limits, in rad).
JOINT_MIN = np.array([-2.7437, -1.7628, -2.8973, -3.0421, -2.8065, 0.5445, -2.8973], np.float32)
JOINT_MAX = np.array([2.7437, 1.7628, 2.8973, -0.1518, 2.8065, 3.7525, 2.8973], np.float32)
# Largest per-step |target - measured| in the 336 demonstrations is 0.26 rad (joint 1); larger
# deltas are clipped and reported.
MAX_JOINT_DELTA = 0.30


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise RuntimeError("Unexpected redirect; refusing to forward the API token")


def encode_rgb(image) -> str:
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError("Expected an RGB uint8 NumPy array shaped [H,W,3]")
    if not 0 < image.shape[0] * image.shape[1] <= 4_000_000:
        raise ValueError("Image exceeds the server's pixel limit")
    buffer = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(image)).save(buffer, format="PNG", compress_level=1)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


class InferenceClient:
    """Sequential, single-session client. Do not share across threads.

    Observation age is measured on the CLIENT monotonic clock (encoding, upload, inference,
    download). Any failure invalidates the session: reset and capture a fresh observation.
    """

    def __init__(self, url, token, timeout=5.0, max_observation_age=0.5, allow_mock=False):
        parsed = urlsplit(url)
        if parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError("Use a plain server URL without credentials, query or fragment")
        if parsed.scheme != "https" and not (parsed.scheme == "http" and parsed.hostname in {"127.0.0.1", "localhost", "::1"}):
            raise ValueError("Use local HTTP through SSH, or authenticated HTTPS")
        if not token or not token.isascii():
            raise ValueError("CACOFLOW_API_TOKEN is required and must be ASCII")
        if any(not math.isfinite(v) or v <= 0 for v in (timeout, max_observation_age)):
            raise ValueError("Timeout and maximum observation age must be finite and positive")
        self.url, self.token = url.rstrip("/"), token
        self.timeout, self.max_age = timeout, max_observation_age
        self.session_id, self.sequence, self.instruction = None, 0, None
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        self.info = self._request("/info")
        if self.info.get("protocol_version") != 1:
            raise RuntimeError("Unsupported server protocol")
        if self.info.get("mock") and not allow_mock:
            raise RuntimeError("Server is mock; pass allow_mock=True for transport tests only")

    def _request(self, path, body=None, timeout=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode("utf-8")
        request = Request(self.url + path, data=data, headers={
            "Authorization": "Bearer " + self.token, "Content-Type": "application/json"})
        try:
            with self.opener.open(request, timeout=timeout or self.timeout) as response:
                raw = response.read(MAX_RESPONSE_BYTES + 1)
                if len(raw) > MAX_RESPONSE_BYTES:
                    raise RuntimeError("Oversized server response")
                result = json.loads(raw)
                if not isinstance(result, dict):
                    raise RuntimeError("Expected a JSON object response")
                return result
        except HTTPError as exc:
            raise RuntimeError(f"Server HTTP {exc.code}: {exc.read(2048).decode('utf-8', 'replace')}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise RuntimeError("Inference connection failed; discard pending actions") from exc

    def reset(self, instruction, trial_id=None, placement=None, appearance="seen"):
        """Start a session (and, with trial_id, a logged trial on the server)."""
        self.session_id = None
        result = self._request("/reset", {"protocol_version": 1, "instruction": instruction, "trial_id": trial_id,
                                          "placement": placement, "appearance": appearance})
        if result.get("model_id") != self.info["model_id"] or not isinstance(result.get("session_id"), str):
            raise RuntimeError("Invalid reset response")
        self.session_id, self.instruction, self.sequence = result["session_id"], result["instruction"], 0
        return result

    def predict(self, agentview_rgb, eye_in_hand_rgb, joint_positions, gripper_opening, *,
                captured_monotonic, observation_timestamp_ns):
        """Full action chunk [H, 8] for this observation; NEVER executes anything.

        captured_monotonic: capture time of the OLDEST of the two frames, in time.monotonic()'s clock.
        joint_positions / gripper_opening: measured together with the frames.
        """
        if self.session_id is None:
            raise RuntimeError("Call reset() before predict()")
        if not math.isfinite(captured_monotonic) or captured_monotonic > time.monotonic():
            raise ValueError("Invalid observation capture time")
        request_id, sequence = uuid.uuid4().hex, self.sequence
        self.sequence += 1
        start = time.monotonic()
        try:
            if start - captured_monotonic >= self.max_age:
                raise RuntimeError("Observation already stale; capture a new one")
            body = {
                "protocol_version": 1, "session_id": self.session_id, "sequence": sequence,
                "request_id": request_id, "observation_timestamp_ns": int(observation_timestamp_ns),
                "images": {"agentview_rgb": encode_rgb(agentview_rgb), "eye_in_hand_rgb": encode_rgb(eye_in_hand_rgb)},
                "robot_state": {"joint_positions": [float(v) for v in np.asarray(joint_positions).reshape(7)],
                                "gripper_opening": float(gripper_opening)},
            }
            remaining = self.max_age - (time.monotonic() - captured_monotonic)
            if remaining <= 0:
                raise RuntimeError("Observation expired while encoding images")
            result = self._request("/predict", body, timeout=min(self.timeout, remaining))
            age = time.monotonic() - captured_monotonic
            if age >= self.max_age:
                raise RuntimeError("Prediction is stale; discard the chunk")
            expected = {"protocol_version": 1, "session_id": self.session_id, "sequence": sequence,
                        "request_id": request_id, "observation_timestamp_ns": int(observation_timestamp_ns),
                        "instruction": self.instruction, "model_id": self.info["model_id"],
                        "mock": self.info["mock"], "action_dtype": "float32"}
            if any(result.get(k) != v for k, v in expected.items()):
                raise RuntimeError("Mismatched response provenance; discard the chunk")
            actions = np.asarray(result["actions"], dtype=np.float32)
            if actions.shape != tuple(self.info["action_shape"]) or not np.isfinite(actions).all():
                raise RuntimeError("Invalid returned action shape or values")
            result["client_roundtrip_ms"] = (time.monotonic() - start) * 1000
            result["client_observation_age_ms"] = age * 1000
            return actions, result
        except Exception:
            self.session_id = None
            raise

    def end_trial(self, success, wrong_object=None, notes=""):
        """Operator's judgement: success True/False (None = undecided), wrong_object on paired tasks."""
        if self.session_id is None:
            raise RuntimeError("No open session")
        result = self._request("/end_trial", {"protocol_version": 1, "session_id": self.session_id,
                                              "success": success, "wrong_object": wrong_object, "notes": notes})
        self.session_id = None
        return result


def action_to_command(row, measured_joints):
    """Action row -> (joint target [7], gripper 0/1, clipped?). Uses the joints measured NOW."""
    row = np.asarray(row, np.float32)
    q = np.asarray(measured_joints, np.float32).reshape(7)
    delta = np.clip(row[:7], -MAX_JOINT_DELTA, MAX_JOINT_DELTA)
    target = np.clip(q + delta, JOINT_MIN, JOINT_MAX)
    clipped = bool(np.any(delta != row[:7]) or np.any(target != q + delta))
    return target, float(row[7] > 0.5), clipped


class RobotInterface:
    """Implement with the lab's robot SDK and cameras (the same ones used for data collection)."""

    def observe(self):
        """-> (agentview_rgb uint8 [H,W,3], wrist_rgb uint8 [H,W,3], joints [7] rad, gripper_opening 0..1,
               captured_monotonic of the older frame, observation_timestamp_ns)"""
        raise NotImplementedError

    def joints(self):
        """Current measured joint positions [7] (rad)."""
        raise NotImplementedError

    def command(self, joint_target, gripper):
        """Send one 7-D joint position target (through the approved interpolating controller) and the
        gripper command 0/1 (send only on change if the gripper call blocks)."""
        raise NotImplementedError

    def hold(self):
        """Hold the current position (on errors, stale observations or the end of a trial)."""
        raise NotImplementedError


def run_trial(client, robot, instruction, *, trial_id=None, placement=None, appearance="seen",
              max_steps=600, execute_steps=None, on_step=None):
    """One trial at 30 Hz: observe -> request a chunk -> execute its rows, re-measuring q at each step.

    Inference is synchronous: the robot holds its last target while a chunk is being computed.
    Stops after max_steps control steps (20 s at 30 Hz by default) or on KeyboardInterrupt (operator stop).
    Returns per-chunk timing; the operator then reports the outcome with client.end_trial(...).
    """
    client.reset(instruction, trial_id=trial_id, placement=placement, appearance=appearance)
    execute_steps = execute_steps or client.info["chunk_size"]
    period, step, timings, clipped_steps = 1.0 / CONTROL_HZ, 0, [], 0
    try:
        while step < max_steps:
            agent, wrist, q, grip, captured, stamp_ns = robot.observe()
            actions, reply = client.predict(agent, wrist, q, grip, captured_monotonic=captured,
                                            observation_timestamp_ns=stamp_ns)
            timings.append({"roundtrip_ms": reply["client_roundtrip_ms"], **reply["timing_ms"]})
            next_tick = time.monotonic()
            for k in range(min(execute_steps, max_steps - step)):
                target, gripper, clipped = action_to_command(actions[k], robot.joints())
                clipped_steps += clipped
                robot.command(target, gripper)
                if on_step is not None:
                    on_step(step, k, actions[k], target, gripper)
                step += 1
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\nOperator stop.")
    finally:
        robot.hold()
    return {"steps": step, "chunks": len(timings), "clipped_steps": clipped_steps, "timing": timings}


class ReplayRobot(RobotInterface):
    """Plays a recorded raw episode as if it were the robot; commands are only compared, never sent.

    The 'measured' joints follow the recording (not the commands), so this checks transport,
    preprocessing and action semantics, not closed-loop behaviour.
    """

    def __init__(self, hdf5_path, episode):
        import h5py

        with h5py.File(hdf5_path, "r") as f:
            g = f[episode]
            self.instruction = str(g.attrs["instruction"])
            t_agent = g["t/agent"][:]
            keep = np.r_[0, np.where(np.diff(t_agent) > 0)[0] + 1]
            t_agent, t_wrist, t_ctrl = t_agent[keep], g["t/wrist"][:], g["t/control"][:]
            self.agent = g["obs/agentview_rgb"][:][keep]
            wrist_idx = np.clip(np.searchsorted(t_wrist, t_agent), 0, len(t_wrist) - 1)
            left = np.clip(wrist_idx - 1, 0, len(t_wrist) - 1)
            wrist_idx = np.where(np.abs(t_wrist[left] - t_agent) <= np.abs(t_wrist[wrist_idx] - t_agent), left, wrist_idx)
            self.wrist = g["obs/eye_in_hand_rgb"][:][wrist_idx]
            ctrl = np.clip(np.searchsorted(t_ctrl, t_agent, side="right") - 1, 0, len(t_ctrl) - 1)
            self.q = g["obs/joint_states"][:][ctrl]
            self.grip = g["obs/gripper_states"][:][ctrl, 0]
            self.recorded = g["actions"][:][ctrl]
        self.i, self.errors, self.grip_ok = 0, [], []

    def __len__(self):
        return len(self.q)

    def observe(self):
        i = min(self.i, len(self) - 1)
        return self.agent[i], self.wrist[i], self.q[i], self.grip[i], time.monotonic(), time.time_ns()

    def joints(self):
        return self.q[min(self.i, len(self) - 1)]

    def command(self, joint_target, gripper):
        i = min(self.i, len(self) - 1)
        self.errors.append(np.abs(joint_target - self.recorded[i, :7]))
        self.grip_ok.append(gripper == float(self.recorded[i, 7] > 0.5))
        self.i += 1

    def hold(self):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="http://127.0.0.1:8000")
    parser.add_argument("--info-only", action="store_true")
    parser.add_argument("--replay", help="Raw recording (HDF5) to play instead of a robot")
    parser.add_argument("--episode", default="episode_000")
    parser.add_argument("--trial-id", default=None, help="Log this replay as a trial on the server")
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-observation-age", type=float, default=0.5,
                        help="Seconds; replay frames are 'captured' at request time")
    parser.add_argument("--allow-mock", action="store_true")
    args = parser.parse_args()
    client = InferenceClient(args.url, os.environ.get("CACOFLOW_API_TOKEN", ""), args.timeout,
                             args.max_observation_age, args.allow_mock)
    print(json.dumps(client.info, indent=2))
    if args.info_only:
        return
    if not args.replay:
        parser.error("Pass --replay <hdf5> to test without a robot (robot use: import run_trial and RobotInterface).")
    robot = ReplayRobot(args.replay, args.episode)
    instruction = client.info["tasks"][0] if client.info["mock"] else robot.instruction
    result = run_trial(client, robot, instruction, trial_id=args.trial_id, placement="replay",
                       max_steps=min(args.max_steps or len(robot), len(robot)))
    if args.trial_id:
        print("Trial log:", client.end_trial(None, None, "replay test (no outcome)")["trial_dir"])
    else:
        client.end_trial(None)
    t = result["timing"]
    errors = np.degrees(np.asarray(robot.errors))
    print(f"{result['steps']} steps in {result['chunks']} chunks, clipped steps: {result['clipped_steps']}")
    print(f"round trip {np.median([x['roundtrip_ms'] for x in t]):.1f} ms median "
          f"(inference {np.median([x['inference'] for x in t]):.1f} ms, decoding {np.median([x['decoding'] for x in t]):.1f} ms)")
    if not client.info["mock"]:
        print(f"joint targets vs. recorded: mean {errors.mean():.3f} deg, max {errors.max():.3f} deg; "
              f"gripper agreement {np.mean(robot.grip_ok):.3f}")


if __name__ == "__main__":
    main()
