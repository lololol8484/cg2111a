#!/usr/bin/env python3
"""
shared_state.py - Shared state between the SLAM worker and GUI.
"""

from __future__ import annotations

import atexit
import ctypes
import multiprocessing
import multiprocessing.shared_memory

from settings import MAP_SIZE_PIXELS, UNKNOWN_BYTE, SCAN_SIZE


class ProcessSharedState:
    def __init__(self) -> None:
        self._cleaned = False

        map_size = MAP_SIZE_PIXELS * MAP_SIZE_PIXELS
        self.shm = multiprocessing.shared_memory.SharedMemory(create=True, size=map_size)
        self.shm.buf[:] = bytes([UNKNOWN_BYTE]) * map_size

        self.x_mm = multiprocessing.Value(ctypes.c_double, 0.0)
        self.y_mm = multiprocessing.Value(ctypes.c_double, 0.0)
        self.theta_deg = multiprocessing.Value(ctypes.c_double, 0.0)

        self.valid_points = multiprocessing.Value(ctypes.c_int, 0)
        self.rounds_seen = multiprocessing.Value(ctypes.c_int, 0)
        self.map_version = multiprocessing.Value(ctypes.c_int, 0)
        self.pose_version = multiprocessing.Value(ctypes.c_int, 0)
        self.scan_version = multiprocessing.Value(ctypes.c_int, 0)

        self.connected = multiprocessing.Value(ctypes.c_bool, False)
        self.stopped = multiprocessing.Value(ctypes.c_bool, False)
        self.paused = multiprocessing.Value(ctypes.c_bool, False)

        self.status_note = multiprocessing.Array(ctypes.c_char, 128)
        self.error_message = multiprocessing.Array(ctypes.c_char, 256)

        self.scan_distances_mm = multiprocessing.Array(ctypes.c_int, SCAN_SIZE)
        self.scan_angles_deg = multiprocessing.Array(ctypes.c_double, SCAN_SIZE)

        self.stop_event = multiprocessing.Event()

        atexit.register(self.cleanup)

    def set_status(self, msg: str) -> None:
        self.status_note.value = msg.encode("utf-8")[:127]

    def get_status(self) -> str:
        return self.status_note.value.decode("utf-8", errors="replace")

    def set_error(self, msg: str) -> None:
        self.error_message.value = msg.encode("utf-8")[:255]

    def get_error(self) -> str:
        raw = self.error_message.value
        return raw.decode("utf-8", errors="replace") if raw else ""

    def set_scan(self, distances, angles) -> None:
        n = min(SCAN_SIZE, len(distances), len(angles))
        for i in range(n):
            self.scan_distances_mm[i] = int(distances[i])
            self.scan_angles_deg[i] = float(angles[i])
        for i in range(n, SCAN_SIZE):
            self.scan_distances_mm[i] = 0
            self.scan_angles_deg[i] = 0.0
        self.scan_version.value += 1

    def get_scan_distances(self):
        return list(self.scan_distances_mm)

    def get_scan_angles(self):
        return list(self.scan_angles_deg)

    def cleanup(self) -> None:
        if self._cleaned:
            return
        self._cleaned = True
        try:
            self.shm.close()
        except Exception:
            pass
        try:
            self.shm.unlink()
        except Exception:
            pass
