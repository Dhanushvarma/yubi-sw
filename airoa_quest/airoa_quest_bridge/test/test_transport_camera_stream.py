"""Tests for the camera-stream wire format and TCP transport (no ROS needed)."""

import socket
import threading
import time

import pytest

from airoa_quest_bridge.transport.camera_stream import (
    CameraFrame,
    CameraStreamTransport,
    HEADER,
    MAX_JPEG_BYTES,
    pack_frame,
    unpack_header,
)


def _frame(jpeg=b"\xff\xd8jpeg\xff\xd9", **overrides):
    values = dict(
        camera_id=0,
        capture_device_ns=1_000_000_000,
        send_device_ns=1_045_000_000,
        width=1280,
        height=960,
        fx=640.0,
        fy=641.0,
        cx=639.5,
        cy=479.5,
        position=(0.03, -0.02, 0.05),
        rotation=(0.0, 0.0, 0.0, 1.0),
        jpeg=jpeg,
        pc_monotonic_ns=0,
    )
    values.update(overrides)
    return CameraFrame(**values)


def test_header_is_80_bytes():
    assert HEADER.size == 80


def test_pack_unpack_round_trip():
    original = _frame()
    data = pack_frame(original)
    parsed, jpeg_len = unpack_header(data[: HEADER.size])

    assert jpeg_len == len(original.jpeg)
    assert data[HEADER.size:] == original.jpeg
    assert parsed.capture_device_ns == original.capture_device_ns
    assert parsed.send_device_ns == original.send_device_ns
    assert (parsed.width, parsed.height) == (1280, 960)
    assert parsed.fx == pytest.approx(640.0)
    assert parsed.cy == pytest.approx(479.5)
    assert parsed.position == pytest.approx((0.03, -0.02, 0.05))
    assert parsed.rotation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_device_latency_never_negative():
    assert _frame().device_latency_ns == 45_000_000
    assert _frame(send_device_ns=1).device_latency_ns == 0


def test_bad_magic_rejected():
    data = bytearray(pack_frame(_frame()))
    data[0:4] = b"NOPE"
    with pytest.raises(ValueError):
        unpack_header(bytes(data[: HEADER.size]))


def test_oversized_jpeg_len_rejected():
    data = bytearray(pack_frame(_frame()))
    data[76:80] = (MAX_JPEG_BYTES + 1).to_bytes(4, "little")
    with pytest.raises(ValueError):
        unpack_header(bytes(data[: HEADER.size]))


def test_transport_receives_frames_from_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    frames = [_frame(jpeg=b"a" * 10), _frame(jpeg=b"b" * 300_000, camera_id=1)]

    def serve():
        conn, _ = server.accept()
        with conn:
            for f in frames:
                # Split writes so the reader has to reassemble partial reads.
                payload = pack_frame(f)
                conn.sendall(payload[:7])
                conn.sendall(payload[7:])
            time.sleep(0.2)

    threading.Thread(target=serve, daemon=True).start()

    received = []
    done = threading.Event()

    def on_frame(frame):
        received.append(frame)
        if len(received) == len(frames):
            done.set()

    transport = CameraStreamTransport(ip="127.0.0.1", tcp_port=port, on_frame=on_frame)
    transport.try_connect()
    assert transport.connected
    assert done.wait(5.0)
    transport.stop()
    server.close()

    assert [f.jpeg for f in received] == [f.jpeg for f in frames]
    assert received[1].camera_id == 1
    assert all(f.pc_monotonic_ns > 0 for f in received)
