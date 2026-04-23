#!/usr/bin/env python3
"""
tcp_client.py  -  Local machine side.

Receives raw LiDAR scan data from the Raspberry Pi over TCP, runs
BreezySLAM locally to build the occupancy map, and displays it with
Matplotlib – the same window as the original ui.py.

Usage:
    python3 tcp_client.py --host <pi-ip-or-hostname> [--port 65432]

Dependencies (install locally):
    pip install matplotlib numpy breezyslam
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from typing import Optional

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Constants 
MAP_SIZE_PIXELS  = 1000
MAP_SIZE_METERS  = 5
MAP_QUALITY      = 7
HOLE_WIDTH_MM    = 100
SCAN_SIZE        = 360
SCAN_RATE_HZ     = 5
DETECTION_ANGLE  = 360
MAX_DISTANCE_MM  = 5000
MIN_VALID_POINTS = 150
UNKNOWN_BYTE     = 127
UI_REFRESH_HZ    = 8


class _SlamState:
    def __init__(self) -> None:
        self._lock = threading.Lock()

        self.map_array: np.ndarray = np.full(
            (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), UNKNOWN_BYTE, dtype=np.uint8
        )
        self.x_mm:         float = 0.0
        self.y_mm:         float = 0.0
        self.theta_deg:    float = 0.0
        self.valid_points: int   = 0
        self.status:       str   = "connecting …"
        self.error:        str   = ""
        self.connected:    bool  = False
        self._dirty_map:   bool  = False

    def update_map(self, mapbytes: bytearray,
                   x_mm: float, y_mm: float, theta_deg: float,
                   valid_points: int) -> None:
        arr = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
            MAP_SIZE_PIXELS, MAP_SIZE_PIXELS
        ).copy()
        with self._lock:
            self.map_array     = arr
            self.x_mm          = x_mm
            self.y_mm          = y_mm
            self.theta_deg     = theta_deg
            self.valid_points  = valid_points
            self._dirty_map    = True

    def update_status(self, status: str, error: str = "") -> None:
        with self._lock:
            self.status = status
            self.error  = error

    def snapshot(self) -> dict:
        with self._lock:
            dirty = self._dirty_map
            self._dirty_map = False
            return {
                "map":          self.map_array.copy() if dirty else None,
                "dirty":        dirty,
                "x_mm":         self.x_mm,
                "y_mm":         self.y_mm,
                "theta_deg":    self.theta_deg,
                "valid_points": self.valid_points,
                "status":       self.status,
                "error":        self.error,
                "connected":    self.connected,
            }

def _slam_loop(host: str, port: int, state: _SlamState,
               stop_event: threading.Event) -> None:
    """Connects to the Pi, receives scan JSON, feeds BreezySLAM, updates state."""
    try:
        from breezyslam.algorithms import RMHC_SLAM
        from breezyslam.sensors import Laser
        import pybreezyslam
    except ImportError:
        state.update_status("error", "breezyslam not installed – run: pip install breezyslam")
        return

    while not stop_event.is_set():
        # try to connect to server side 
        try:
            sock = socket.create_connection((host, port), timeout=5.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(10.0)
        except (ConnectionRefusedError, TimeoutError, OSError) as exc:
            state.connected = False
            state.update_status("disconnected – retrying …", str(exc))
            print(f"[client] could not connect: {exc} – retrying in 2 s")
            time.sleep(2.0)
            continue

        state.connected = True
        state.update_status("connected – initialising SLAM …")
        print(f"[client] connected to {host}:{port}")

        laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
        # we want the starting position of the robot to be plotted at the bottom center
        start_x_mm = (MAP_SIZE_METERS * 1000) * 0.05        # center
        start_y_mm = (MAP_SIZE_METERS * 1000) / 2     # buffer from the bottom edge

        laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
        slam  = RMHC_SLAM(
            laser,
            MAP_SIZE_PIXELS,
            MAP_SIZE_METERS,
            hole_width_mm=HOLE_WIDTH_MM,
            map_quality=MAP_QUALITY,
        )

        # directly access and hardcode the starting position
        slam.position =  pybreezyslam.Position(start_x_mm, start_y_mm, 0)
        mapbytes: bytearray = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        previous_distances: Optional[list[int]] = None

        # receving information from the server side
        buf = b""
        try:
            while not stop_event.is_set():
                chunk = sock.recv(65536)
                if not chunk:
                    break
                buf += chunk

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError:
                        continue

                    if msg.get("type") == "status":
                        state.update_status(msg.get("status", ""), msg.get("error", ""))
                        continue

                    distances: list[int]   = msg["distances_mm"]
                    angles:    list[float] = msg["angles_deg"]
                    valid:     int         = msg["valid_points"]
                    round_num: int         = msg.get("round", 0)

                    if valid >= MIN_VALID_POINTS:
                        slam.update(distances, scan_angles_degrees=angles)
                        previous_distances = list(distances)
                        note = f"live (round {round_num}, {valid} pts)"
                    elif previous_distances is not None:
                        slam.update(previous_distances, scan_angles_degrees=angles)
                        note = f"reusing prev (round {round_num}, {valid} pts)"
                    else:
                        state.update_status(f"waiting for valid scan ({valid} pts)")
                        continue

                    x_mm, y_mm, theta_deg = slam.getpos()
                    slam.getmap(mapbytes)

                    state.update_map(mapbytes, x_mm, y_mm, theta_deg, valid)
                    state.update_status(note)

        except (ConnectionResetError, TimeoutError, OSError) as exc:
            print(f"[client] connection lost: {exc}")
        finally:
            sock.close()
            state.connected = False
            state.update_status("disconnected – retrying …")

    print("[client] SLAM thread exiting")


class SlamViewer:
    def __init__(self, host: str, port: int) -> None:
        self.state      = _SlamState()
        self.stop_event = threading.Event()
        self._closed    = False
        self._anim: Optional[FuncAnimation] = None
        self.robot_heading = None

        self._slam_thread = threading.Thread(
            target=_slam_loop,
            args=(host, port, self.state, self.stop_event),
            daemon=True,
            name="slam-thread",
        )
        self._slam_thread.start()

        self.fig, self.ax_map = plt.subplots(figsize=(10, 8))
        self.fig.canvas.mpl_connect("close_event", self._on_close)

        self.ax_map.set_title("SLAM Map")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.set_xlim(0, MAP_SIZE_METERS)
        self.ax_map.set_ylim(0, MAP_SIZE_METERS)
        self.ax_map.set_aspect("equal")

        self.map_image = self.ax_map.imshow(
            np.full((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), UNKNOWN_BYTE, dtype=np.uint8),
            cmap="gray",
            origin="lower",
            extent=[0, MAP_SIZE_METERS, 0, MAP_SIZE_METERS],
            vmin=0,
            vmax=255,
        )
        self.robot_point, = self.ax_map.plot([], [], "bo", markersize=8)
        self.info_text = self.ax_map.text(
            0.02, 0.98, "",
            transform=self.ax_map.transAxes,
            va="top",
            bbox=dict(facecolor="white", alpha=0.8),
        )

    def _on_close(self, _event) -> None:
        self.close()

    def start(self) -> None:
        interval_ms = max(50, int(1000 / max(UI_REFRESH_HZ, 1)))
        self._anim = FuncAnimation(
            self.fig, self._update,
            interval=interval_ms,
            cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show(block=True)

    def _update(self, _frame) -> None:
        if self._closed:
            return

        snap = self.state.snapshot()

        if snap["error"]:
            self.fig.suptitle(f"ERROR: {snap['error']}", color="red")
        else:
            prefix = "" if snap["connected"] else "[disconnected]  "
            self.fig.suptitle(prefix + (snap["status"] or "SLAM running"))

        if snap["dirty"] and snap["map"] is not None:
            self.map_image.set_data(snap["map"])

        x_m       = snap["x_mm"] / 1000.0
        y_m       = snap["y_mm"] / 1000.0
        theta_deg = snap["theta_deg"]

        self.robot_point.set_data([x_m], [y_m])

        if self.robot_heading is not None:
            self.robot_heading.remove()
            self.robot_heading = None

        if x_m != 0.0 or y_m != 0.0:
            theta_rad = np.deg2rad(theta_deg)
            self.robot_heading = self.ax_map.arrow(
                x_m, y_m,
                0.18 * np.cos(theta_rad),
                0.18 * np.sin(theta_rad),
                head_width=0.06, head_length=0.09,
                fc="c", ec="c",
            )

        self.info_text.set_text(
            f"x={x_m:.2f}m  y={y_m:.2f}m\n"
            f"θ={theta_deg:.1f}°  valid={snap['valid_points']}"
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.stop_event.set()
        self._slam_thread.join(timeout=3.0)
        

def main() -> None:
    viewer = SlamViewer("100.68.229.80",12345)
    viewer.start()


if __name__ == "__main__":
    main()
