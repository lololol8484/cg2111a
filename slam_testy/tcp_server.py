#!/usr/bin/env python3
"""
tcp_server.py  -  Raspberry Pi side.

Connects to the RPLidar, resamples each 360-degree scan into a fixed-size
array, and streams the raw scan data to a remote client over TCP.

Wire protocol  (newline-terminated JSON, one message per scan round):
    {
        "distances_mm": [<int>, ...],   # SCAN_SIZE elements
        "angles_deg":   [<float>, ...], # SCAN_SIZE elements
        "valid_points": <int>,
        "round":        <int>
    }

A special status message is sent on connect and on error:
    { "type": "status", "status": <str>, "error": <str> }
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from settings import (
    SCAN_SIZE, DETECTION_ANGLE, MAX_DISTANCE_MM,
    LIDAR_OFFSET_DEG, MIN_VALID_POINTS, INITIAL_ROUNDS_SKIP,
    LIDAR_PORT,
)
import lidar as lidar_driver

_SCAN_ANGLES: list[float] = [
    float(i * DETECTION_ANGLE / SCAN_SIZE)
    for i in range(SCAN_SIZE)
]


def _resample_scan(
    raw_angles: list[float],
    raw_distances: list[float],
) -> tuple[list[int], int]:
    """Bin raw LiDAR measurements into SCAN_SIZE fixed angular slots."""
    bin_sums   = [0.0] * SCAN_SIZE
    bin_counts = [0]   * SCAN_SIZE

    for angle, dist in zip(raw_angles, raw_distances):
        if dist <= 0:
            continue
        ccw_angle = -angle + LIDAR_OFFSET_DEG
        bin_idx   = int(round(ccw_angle)) % SCAN_SIZE
        bin_sums[bin_idx]   += dist
        bin_counts[bin_idx] += 1

    scan_distances: list[int] = []
    valid = 0
    for i in range(SCAN_SIZE):
        if bin_counts[i] > 0:
            avg = bin_sums[i] / bin_counts[i]
            if avg >= MAX_DISTANCE_MM:
                scan_distances.append(MAX_DISTANCE_MM)
            else:
                scan_distances.append(int(avg))
                valid += 1
        else:
            scan_distances.append(MAX_DISTANCE_MM)

    return scan_distances, valid


def _send_json(conn: socket.socket, obj: dict) -> bool:
    try:
        payload = json.dumps(obj, separators=(",", ":")) + "\n"
        conn.sendall(payload.encode("utf-8"))
        return True
    except (BrokenPipeError, ConnectionResetError, OSError):
        return False


def _serve_client(conn: socket.socket, lidar, scan_mode: int) -> None:
    """Stream resampled scans to one connected client."""
    print("[server] client connected – streaming scans")
    _send_json(conn, {"type": "status", "status": "connected", "error": ""})

    round_num = 0
    try:
        for raw_angles, raw_distances in lidar_driver.scan_rounds(lidar, scan_mode):
            round_num += 1

            if round_num <= INITIAL_ROUNDS_SKIP:
                _send_json(conn, {
                    "type":   "status",
                    "status": f"warming up {round_num}/{INITIAL_ROUNDS_SKIP}",
                    "error":  "",
                })
                continue

            distances, valid = _resample_scan(raw_angles, raw_distances)

            ok = _send_json(conn, {
                "distances_mm": distances,
                "angles_deg":   _SCAN_ANGLES,
                "valid_points": valid,
                "round":        round_num,
            })
            if not ok:
                break  

    except Exception as exc:
        _send_json(conn, {"type": "status", "status": "error", "error": str(exc)})
        print(f"[server] scan error: {exc}")
    finally:
        conn.close()
        print("[server] client disconnected")


def main() -> None:
    print(f"[server] connecting to LiDAR on {LIDAR_PORT} …")
    lidar = lidar_driver.connect()
    if lidar is None:
        sys.exit(f"[server] could not connect to LiDAR on {LIDAR_PORT}")

    scan_mode = lidar_driver.get_scan_mode(lidar)
    print(f"[server] LiDAR ready (scan mode {scan_mode})")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # ip addres is client side tailscale ip
        srv.bind(("100.68.229.80",12345))
        srv.listen(1)

        try:
            while True:
                print("[server] waiting for client …")
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"[server] connection from {addr}")
                _serve_client(conn, lidar, scan_mode)
        except KeyboardInterrupt:
            print("\n[server] shutting down")
        finally:
            lidar_driver.disconnect(lidar)


if __name__ == "__main__":
    main()