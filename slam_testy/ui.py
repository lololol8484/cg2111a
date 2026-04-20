	#!/usr/bin/env python3
"""
ui.py - GUI-only Matplotlib SLAM viewer with one large SLAM map.
"""

from __future__ import annotations

import atexit
import multiprocessing
import os
from typing import Optional

import matplotlib

if not os.environ.get("DISPLAY"):
    raise SystemExit(
        "[slam] No graphical display detected.\n"
        "Run this from the Raspberry Pi desktop, VNC, or ssh -X."
    )

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from settings import MAP_SIZE_PIXELS, MAP_SIZE_METERS, UI_REFRESH_HZ
from shared_state import ProcessSharedState
from slam_process import run_slam_process


class SlamViewer:
    def __init__(self) -> None:
        self.pss = ProcessSharedState()
        self.slam_proc: Optional[multiprocessing.Process] = None
        self._closed = False
        self._anim: Optional[FuncAnimation] = None
        self.robot_heading = None

        atexit.register(self.close)

        self.fig, self.ax_map = plt.subplots(figsize=(10, 8))
        self.fig.canvas.mpl_connect("close_event", self._on_close)

        self.ax_map.set_title("SLAM Map")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.set_xlim(0, MAP_SIZE_METERS)
        self.ax_map.set_ylim(0, MAP_SIZE_METERS)
        self.ax_map.set_aspect("equal")

        self.map_image = self.ax_map.imshow(
            np.full((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), 127, dtype=np.uint8),
            cmap="gray",
            origin="lower",
            extent=[0, MAP_SIZE_METERS, 0, MAP_SIZE_METERS],
            vmin=0,
            vmax=255,
        )

        self.robot_point, = self.ax_map.plot([], [], "bo", markersize=8)

        self.info_text = self.ax_map.text(
            0.02,
            0.98,
            "",
            transform=self.ax_map.transAxes,
            va="top",
            bbox=dict(facecolor="white", alpha=0.8),
        )

    def _on_close(self, event) -> None:
        self.close()

    def start(self) -> None:
        try:
            self.slam_proc = multiprocessing.Process(
                target=run_slam_process,
                args=(self.pss,),
                name="slam-process",
                daemon=True,
            )
            self.slam_proc.start()

            interval_ms = max(50, int(1000 / max(UI_REFRESH_HZ, 1)))
            self._anim = FuncAnimation(
                self.fig,
                self._update,
                interval=interval_ms,
                cache_frame_data=False,
            )

            plt.tight_layout()
            plt.show(block=True)
        finally:
            self.close()

    def _update(self, _frame):
        if self._closed:
            return

        err = self.pss.get_error()
        if err:
            self.fig.suptitle(f"ERROR: {err}", color="red")
        else:
            self.fig.suptitle(self.pss.get_status() or "SLAM running")

        map_array = np.frombuffer(self.pss.shm.buf, dtype=np.uint8).copy().reshape(
            MAP_SIZE_PIXELS, MAP_SIZE_PIXELS
        )
        self.map_image.set_data(map_array)

        x_m = self.pss.x_mm.value / 1000.0
        y_m = self.pss.y_mm.value / 1000.0
        theta_deg = self.pss.theta_deg.value

        self.robot_point.set_data([x_m], [y_m])

        if self.robot_heading is not None:
            self.robot_heading.remove()

        theta_rad = np.deg2rad(theta_deg)
        dx = 0.18 * np.cos(theta_rad)
        dy = 0.18 * np.sin(theta_rad)
        self.robot_heading = self.ax_map.arrow(
            x_m,
            y_m,
            dx,
            dy,
            head_width=0.06,
            head_length=0.09,
            fc="c",
            ec="c",
        )

        self.info_text.set_text(
            f"x={x_m:.2f}m  y={y_m:.2f}m\n"
            f"θ={theta_deg:.1f}°  valid={self.pss.valid_points.value}"
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True

        try:
            self.pss.stop_event.set()
        except Exception:
            pass

        try:
            if self.slam_proc is not None and self.slam_proc.is_alive():
                self.slam_proc.join(timeout=2.0)
            if self.slam_proc is not None and self.slam_proc.is_alive():
                self.slam_proc.terminate()
                self.slam_proc.join(timeout=1.0)
        except Exception:
            pass

        try:
            self.pss.cleanup()
        except Exception:
            pass


def run() -> None:
    viewer = SlamViewer()
    viewer.start()
