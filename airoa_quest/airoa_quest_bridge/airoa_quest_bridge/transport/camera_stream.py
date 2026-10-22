"""TCP transport for the yubi-quest-app passthrough-camera stream.

The Quest app is the TCP server (default port 65433); the PC connects to it,
as with the pose stream. Nothing is sent PC -> Quest. Each frame is an 80-byte
little-endian header followed by ``jpeg_len`` bytes of baseline JPEG, rows
ordered top to bottom:

  offset  type     field
  0       4s       magic b"YQCF"
  4       u16      version (1)
  6       u16      camera_id (0 = left passthrough camera, 1 = right)
  8       u64      capture_device_ns   Quest clock at exposure
  16      u64      send_device_ns      same clock, when the frame is written
  24      u32      width               pixels of this JPEG
  28      u32      height
  32      f32 x4   fx, fy, cx, cy      pinhole intrinsics for width x height
  48      f32 x3   camera position relative to the HMD (Unity frame, metres)
  60      f32 x4   camera rotation relative to the HMD (Unity frame, x y z w);
                   camera local +Z is the optical axis, +X image right, +Y up
  76      u32      jpeg_len

Only ``send_device_ns - capture_device_ns`` is used for stamping, so the Quest
clock never has to be synchronised with the PC clock.
"""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import threading
import time
from typing import Any, Callable, List, Optional, Tuple

MAGIC = b"YQCF"
VERSION = 1
HEADER = struct.Struct("<4sHHQQII4f3f4fI")
assert HEADER.size == 80

# Upper bound on one JPEG; a larger length means a corrupt or foreign stream.
MAX_JPEG_BYTES = 16 * 1024 * 1024


@dataclass
class CameraFrame:
    camera_id: int
    capture_device_ns: int
    send_device_ns: int
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    position: Tuple[float, float, float]  # Unity frame, relative to the HMD
    rotation: Tuple[float, float, float, float]  # Unity frame, x y z w
    jpeg: bytes
    pc_monotonic_ns: int  # time.monotonic_ns() when the frame was fully received

    @property
    def device_latency_ns(self) -> int:
        """Exposure -> send delay measured on the Quest (never negative)."""
        return max(0, self.send_device_ns - self.capture_device_ns)


def pack_frame(frame: CameraFrame) -> bytes:
    """Serialise a frame in the wire format (used by tests and tools)."""
    header = HEADER.pack(
        MAGIC,
        VERSION,
        frame.camera_id,
        frame.capture_device_ns,
        frame.send_device_ns,
        frame.width,
        frame.height,
        frame.fx,
        frame.fy,
        frame.cx,
        frame.cy,
        *frame.position,
        *frame.rotation,
        len(frame.jpeg),
    )
    return header + frame.jpeg


def unpack_header(data: bytes) -> Tuple[CameraFrame, int]:
    """Parse an 80-byte header; return a frame without JPEG data and jpeg_len.

    Raises ValueError for a wrong magic, an unknown version or a jpeg_len
    above MAX_JPEG_BYTES.
    """
    fields = HEADER.unpack(data)
    magic, version, camera_id, capture_ns, send_ns, width, height = fields[:7]
    fx, fy, cx, cy = fields[7:11]
    position = tuple(fields[11:14])
    rotation = tuple(fields[14:18])
    jpeg_len = fields[18]
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r}")
    if version != VERSION:
        raise ValueError(f"unsupported version {version}")
    if jpeg_len > MAX_JPEG_BYTES:
        raise ValueError(f"jpeg_len {jpeg_len} exceeds {MAX_JPEG_BYTES}")
    frame = CameraFrame(
        camera_id=camera_id,
        capture_device_ns=capture_ns,
        send_device_ns=send_ns,
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        position=position,
        rotation=rotation,
        jpeg=b"",
        pc_monotonic_ns=0,
    )
    return frame, jpeg_len


class CameraStreamTransport:
    """Connect to the Quest camera server and decode frames.

    Threading model mirrors TcpJsonTransport: the node calls
    :meth:`try_connect` from a ROS timer; once connected, one daemon thread
    reads frames and invokes ``on_frame`` from that thread.
    """

    def __init__(
        self,
        *,
        ip: str,
        tcp_port: int,
        on_frame: Callable[[CameraFrame], None],
        logger: Any = None,
        fps_window: int = 30,
    ) -> None:
        self._ip = ip
        self._tcp_port = int(tcp_port)
        self._on_frame = on_frame
        self._log = logger
        self._fps_window = int(fps_window)

        self._lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._connected = False
        self._frame_times: List[float] = []

    # ---------- public API ----------

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def fps(self) -> float:
        with self._lock:
            times = tuple(self._frame_times)
        if len(times) >= 2 and times[-1] > times[0]:
            return (len(times) - 1) / (times[-1] - times[0])
        return 0.0

    def try_connect(self) -> None:
        """Attempt one TCP connect. No-op if already connected."""
        with self._lock:
            if self._connected:
                return
        if not self._ip:
            return

        sock: Optional[socket.socket] = None
        try:
            self._info(f"Connecting camera TCP {self._ip}:{self._tcp_port} ...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            sock.settimeout(2.0)
            sock.connect((self._ip, self._tcp_port))
            # A stalled stream (app paused) must not block forever.
            sock.settimeout(5.0)

            with self._lock:
                self._sock = sock
                self._connected = True
                self._frame_times.clear()

            threading.Thread(target=self._recv_loop, args=(sock,), daemon=True).start()
            self._info("Quest camera connected")
        except Exception as e:
            self._warn(f"Camera connect failed: {e}")
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

    def stop(self) -> None:
        self._disconnect("transport stop")

    # ---------- internals ----------

    def _disconnect(self, reason: str) -> None:
        with self._lock:
            was_connected = self._connected
            self._connected = False
            sock = self._sock
            self._sock = None

        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

        if was_connected:
            self._warn(f"Quest camera disconnected: {reason}")

    def _recv_loop(self, sock: socket.socket) -> None:
        while True:
            try:
                frame, jpeg_len = unpack_header(_recv_exact(sock, HEADER.size))
                frame.jpeg = _recv_exact(sock, jpeg_len)
            except Exception as e:
                self._disconnect(f"camera stream error: {e}")
                return

            frame.pc_monotonic_ns = time.monotonic_ns()
            with self._lock:
                if self._sock is not sock:
                    return
                self._frame_times.append(frame.pc_monotonic_ns / 1e9)
                if len(self._frame_times) > self._fps_window:
                    self._frame_times.pop(0)

            try:
                self._on_frame(frame)
            except Exception as e:
                self._warn(f"on_frame callback raised: {e}")

    # ---------- logging helpers ----------

    def _info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)

    def _warn(self, msg: str) -> None:
        if self._log is not None:
            self._log.warn(msg)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("server closed")
        buf.extend(chunk)
    return bytes(buf)
