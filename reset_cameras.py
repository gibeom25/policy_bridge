#!/usr/bin/env python3
"""Hardware-reset the RealSense cameras used by robot_server.py and wait until both are back.

Exit code 0 = both cameras re-enumerated, 1 = a camera is missing.
"""
import os
import sys
import time

import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_server import AGENT_SERIAL, WRIST_SERIAL  # noqa: E402

WANT = {AGENT_SERIAL: "agent", WRIST_SERIAL: "wrist"}


def serials() -> set[str]:
    return {d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()}


def main() -> int:
    devs = list(rs.context().query_devices())
    if not devs:
        print("[reset_cameras] no RealSense camera found", flush=True)
        return 1
    for d in devs:
        sn = d.get_info(rs.camera_info.serial_number)
        print(f"[reset_cameras] hardware reset {WANT.get(sn, '?')} ({d.get_info(rs.camera_info.name)}, {sn})", flush=True)
        d.hardware_reset()
    time.sleep(3.0)  # devices drop off the bus first, then re-enumerate
    t0 = time.time()
    have: set[str] = set()
    while time.time() - t0 < 25.0:
        have = serials()
        if set(WANT) <= have:
            time.sleep(1.0)  # let the firmware finish booting before the server opens the streams
            print(f"[reset_cameras] cameras back after {time.time() - t0 + 3:.1f}s: "
                  + ", ".join(f"{WANT[s]}={s}" for s in WANT), flush=True)
            return 0
        time.sleep(0.5)
    missing = [f"{WANT[s]}={s}" for s in WANT if s not in have]
    print(f"[reset_cameras] ERROR: camera(s) did not come back: {missing} (check the USB cables)", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
