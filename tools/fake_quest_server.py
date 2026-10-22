#!/usr/bin/env python3
"""Stand-in for the Quest app, for exercising the quest_only stack without a headset.

Serves the same three channels as yubi-quest-app:
  * TCP 65432: newline-delimited JSON pose frames (the stock app's keys plus
    {left,right}{Trigger,Grip}Value), at --rate Hz. The controllers trace
    circles in front of the HMD and the triggers sweep 0..1.
  * UDP 42000: replies to the bridge's time-sync probes.
  * TCP 65433: passthrough-camera frames in the quest_camera_node wire format
    (airoa_quest_bridge/transport/camera_stream.py), at --camera-fps. Frames
    are synthetic JPEGs (needs Pillow); --no-camera disables the channel.

Run it in its own container on a bridge network, so its UDP 42000 does not
collide with the bridge's on the host, and set quest_ip to the container IP:

  docker run --rm -d --name fake_quest -v "$PWD":/work -w /work python:3.12-slim \\
    sh -c "pip install -q pillow && python tools/fake_quest_server.py"
  docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' fake_quest
"""
from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "airoa_quest" / "airoa_quest_bridge"))
from airoa_quest_bridge.transport.camera_stream import CameraFrame, pack_frame  # noqa: E402

SYNC_REQUEST = struct.Struct("<BQ")
SYNC_REPLY = struct.Struct("<BQQ")


def pose_frame(t: float) -> dict:
    """One JSON frame at time t (seconds); Unity frame (X right, Y up, Z forward)."""
    w = 2.0 * math.pi * 0.25  # one circle every 4 s
    left = {"x": -0.2 + 0.1 * math.cos(w * t), "y": 1.2 + 0.1 * math.sin(w * t), "z": 0.4}
    right = {"x": 0.2 + 0.1 * math.cos(w * t), "y": 1.2 - 0.1 * math.sin(w * t), "z": 0.4}
    identity = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    trigger = 0.5 + 0.5 * math.sin(w * t)
    frame = {
        "ovrTimeNs": time.monotonic_ns(),
        "hmdPosition": {"x": 0.0, "y": 1.6, "z": 0.0},
        "hmdRotation": identity,
        "leftControllerPosition": left,
        "leftControllerRotation": identity,
        "rightControllerPosition": right,
        "rightControllerRotation": identity,
        "leftTracked": True,
        "leftValid": True,
        "rightTracked": True,
        "rightValid": True,
        "leftJoystick": {"x": 0.0, "y": 0.0},
        "rightJoystick": {"x": 0.0, "y": 0.0},
        "leftTriggerValue": trigger,
        "rightTriggerValue": 1.0 - trigger,
        "leftGripValue": 0.0,
        "rightGripValue": 0.0,
        "leftTriggerPressed": trigger > 0.5,
        "rightTriggerPressed": trigger < 0.5,
        "hmdBattPct": 90,
        "leftBattPct": 80,
        "rightBattPct": 80,
        "hmdCharging": False,
    }
    for key in (
        "buttonXPressed", "buttonYPressed", "buttonXTouched", "buttonYTouched",
        "startPressed", "buttonAPressed", "buttonBPressed", "buttonATouched",
        "buttonBTouched", "backPressed", "leftThumbstickClick", "rightThumbstickClick",
        "leftThumbstickTouched", "rightThumbstickTouched", "leftIndexTriggerTouched",
        "rightIndexTriggerTouched", "leftGripPressed", "rightGripPressed",
    ):
        frame[key] = False
    return frame


def serve_tcp(port: int, name: str, stream) -> None:
    """Accept one client at a time and run stream(conn) until it disconnects."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"[{name}] listening on TCP {port}", flush=True)
    while True:
        conn, addr = server.accept()
        print(f"[{name}] client {addr}", flush=True)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            stream(conn)
        except OSError as e:
            print(f"[{name}] client gone: {e}", flush=True)
        finally:
            conn.close()


def pose_stream(rate: float):
    def stream(conn: socket.socket) -> None:
        t0 = time.monotonic()
        period = 1.0 / rate
        n = 0
        while True:
            conn.sendall((json.dumps(pose_frame(time.monotonic() - t0)) + "\n").encode())
            n += 1
            time.sleep(max(0.0, t0 + n * period - time.monotonic()))
    return stream


def camera_stream(fps: float, width: int, height: int, latency_ms: float):
    from PIL import Image, ImageDraw

    def render(i: int) -> bytes:
        img = Image.new("RGB", (width, height), (40, 40, 40))
        draw = ImageDraw.Draw(img)
        x = int((i * 8) % width)
        draw.rectangle([x, height // 3, x + width // 8, 2 * height // 3], fill=(230, 120, 30))
        draw.text((10, 10), f"frame {i}", fill=(255, 255, 255))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        return buf.getvalue()

    def stream(conn: socket.socket) -> None:
        t0 = time.monotonic()
        period = 1.0 / fps
        i = 0
        while True:
            jpeg = render(i)
            now = time.monotonic_ns()
            frame = CameraFrame(
                camera_id=0,
                capture_device_ns=now - int(latency_ms * 1e6),
                send_device_ns=now,
                width=width,
                height=height,
                fx=0.8 * width,
                fy=0.8 * width,
                cx=(width - 1) / 2.0,
                cy=(height - 1) / 2.0,
                position=(0.032, -0.02, 0.06),
                rotation=(0.0, 0.0, 0.0, 1.0),
                jpeg=jpeg,
                pc_monotonic_ns=0,
            )
            conn.sendall(pack_frame(frame))
            i += 1
            time.sleep(max(0.0, t0 + i * period - time.monotonic()))
    return stream


def serve_time_sync(port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[sync] listening on UDP {port}", flush=True)
    while True:
        data, addr = sock.recvfrom(64)
        if len(data) != SYNC_REQUEST.size:
            continue
        msg_id, t1 = SYNC_REQUEST.unpack(data)
        if msg_id == 1:
            sock.sendto(SYNC_REPLY.pack(2, t1, time.monotonic_ns()), addr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pose-port", type=int, default=65432)
    parser.add_argument("--camera-port", type=int, default=65433)
    parser.add_argument("--sync-port", type=int, default=42000)
    parser.add_argument("--rate", type=float, default=72.0, help="pose frames per second")
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--latency-ms", type=float, default=40.0,
                        help="simulated exposure-to-send delay")
    parser.add_argument("--no-camera", action="store_true")
    args = parser.parse_args()

    threads = [
        threading.Thread(target=serve_time_sync, args=(args.sync_port,), daemon=True),
        threading.Thread(target=serve_tcp, args=(args.pose_port, "pose", pose_stream(args.rate)),
                         daemon=True),
    ]
    if not args.no_camera:
        stream = camera_stream(args.camera_fps, args.width, args.height, args.latency_ms)
        threads.append(threading.Thread(target=serve_tcp,
                                        args=(args.camera_port, "camera", stream), daemon=True))
    for t in threads:
        t.start()
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
