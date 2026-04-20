#!/usr/bin/env python3
"""
slam_process.py - SLAM background process.

This module contains the function that runs in a dedicated child process to
perform SLAM.  Running SLAM in a separate process gives it its own Python GIL,
so heavy LIDAR reading and map-building never stalls the terminal UI.

The process reads LIDAR scans, resamples them into fixed-size angle bins, and
feeds them into BreezySLAM's RMHC_SLAM algorithm.  The resulting robot pose
and occupancy map are written into a ProcessSharedState object that the UI
process can read at any time.

Scan resampling
---------------
The RPLidar reports measurements at irregular angles.  BreezySLAM expects a
fixed array of SCAN_SIZE evenly-spaced readings.  The resampling step bins raw
readings by their rounded angle (0-359 degrees) and averages any multiple
readings that fall in the same bin.  Bins with no readings are filled with
MAX_DISTANCE_MM (treated as "no obstacle detected").
"""

from __future__ import annotations

import time
from typing import Optional

from settings import (
    SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM,
    MAP_SIZE_PIXELS, MAP_SIZE_METERS, HOLE_WIDTH_MM, MAP_QUALITY,
    LIDAR_OFFSET_DEG, MIN_VALID_POINTS, INITIAL_ROUNDS_SKIP,
    MAP_UPDATE_INTERVAL,
)
from shared_state import ProcessSharedState


# Pre-compute the angle array once so we do not recreate it every scan.
# _SCAN_ANGLES[i] is the angle (degrees) of bin i as seen by BreezySLAM.
# No offset is applied here; the offset is applied during resampling instead,
# so that bin i's distance always corresponds to _SCAN_ANGLES[i].
_SCAN_ANGLES: list[float] = [
    float(i * DETECTION_ANGLE / SCAN_SIZE)
    for i in range(SCAN_SIZE)
]


# Add these constants just above _resample_scan
_MIN_DIST_MM    = 150    # A1M8 blind spot — discard anything closer
_SPIKE_WINDOW   = 5      # bins either side to check for spikes
_SPIKE_RATIO    = 2.5    # bin is a spike if > 2.5× or < 1/2.5× local median
_MAX_INTERP_GAP = 4      # interpolate empty runs of ≤ this many bins


def _resample_scan(
    raw_angles: list[float],
    raw_distances: list[float],
) -> tuple[list[int], int]:
    # ── 1. Bin with median (robust) instead of mean (noisy) ──────────────
    bin_readings: list[list[float]] = [[] for _ in range(SCAN_SIZE)]
    for angle, dist in zip(raw_angles, raw_distances):
        if dist < _MIN_DIST_MM or dist >= MAX_DISTANCE_MM:
            continue                        # gate: blind spot + max range
        ccw_angle = -angle + LIDAR_OFFSET_DEG
        bin_idx   = int(round(ccw_angle)) % SCAN_SIZE
        bin_readings[bin_idx].append(dist)

    bin_dist: list[float] = [0.0] * SCAN_SIZE
    filled:   list[bool]  = [False]  * SCAN_SIZE
    for i in range(SCAN_SIZE):
        if bin_readings[i]:
            s            = sorted(bin_readings[i])
            bin_dist[i]  = s[len(s) // 2]  # median
            filled[i]    = True

    # ── 2. Spike filter ───────────────────────────────────────────────────
    for i in range(SCAN_SIZE):
        if not filled[i]:
            continue
        nbrs = [bin_dist[(i + o) % SCAN_SIZE]
                for o in range(-_SPIKE_WINDOW, _SPIKE_WINDOW + 1)
                if o != 0 and filled[(i + o) % SCAN_SIZE]]
        if len(nbrs) < 3:
            continue
        nbrs.sort()
        med = nbrs[len(nbrs) // 2]
        if med > 0 and (bin_dist[i] > med * _SPIKE_RATIO
                        or bin_dist[i] < med / _SPIKE_RATIO):
            filled[i] = False               # mark spike as invalid

    # ── 3. Build output — empty bins are 0, NOT MAX_DISTANCE_MM ──────────
    #    0  → BreezySLAM skips the bin entirely (no map erasure)
    #    MAX → BreezySLAM fires a 12 m free-space ray (erases walls!)
    scan_distances = [0] * SCAN_SIZE
    valid = 0
    filled_list = [i for i in range(SCAN_SIZE) if filled[i]]
    for i in filled_list:
        scan_distances[i] = int(bin_dist[i])
        valid += 1

    # ── 4. Interpolate short gaps between real obstacle readings ──────────
    n = len(filled_list)
    for k in range(n):
        i0  = filled_list[k]
        i1  = filled_list[(k + 1) % n]
        gap = (i1 - i0) % SCAN_SIZE
        if 1 < gap <= _MAX_INTERP_GAP:
            v0, v1 = bin_dist[i0], bin_dist[i1]
            for g in range(1, gap):
                t = g / gap
                scan_distances[(i0 + g) % SCAN_SIZE] = int(v0*(1-t) + v1*t)

    return scan_distances, valid


def run_slam_process(pss: ProcessSharedState) -> None:
    """Entry point for the SLAM child process.

    Connects to the LIDAR, initialises BreezySLAM, then loops:
      1. Read one full 360-degree LIDAR scan.
      2. Resample it into SCAN_SIZE equal-angle bins.
      3. Feed the resampled scan to RMHC_SLAM.update().
      4. Read back the updated pose and (at a throttled rate) the map.
      5. Write both into pss so the UI can pick them up.

    The loop exits when pss.stop_event is set (by the UI on quit).

    Parameters
    ----------
    pss : ProcessSharedState instance created by the UI process.
          Must be passed as an argument (not captured in a closure) so that
          multiprocessing can transfer the handles to the child process.
    """
    # Import BreezySLAM here so import errors are reported cleanly.
    try:
        from breezyslam.algorithms import RMHC_SLAM
        from breezyslam.sensors import Laser
    except ImportError:
        pss.set_error('BreezySLAM not installed. Run: bash install_slam.sh')
        pss.stopped.value = True
        return

    # Import the LIDAR driver from the same directory.
    try:
        import lidar as lidar_driver
    except ImportError:
        pss.set_error('lidar.py not found in the slam/ directory')
        pss.stopped.value = True
        return

    # Connect to the LIDAR.
    lidar = lidar_driver.connect()
    if lidar is None:
        from settings import LIDAR_PORT
        pss.set_error(f'Could not connect to LIDAR on {LIDAR_PORT}')
        pss.stopped.value = True
        return

    scan_mode = lidar_driver.get_scan_mode(lidar)

    # Initialise SLAM.
    # Laser() describes the sensor: (num_bins, scan_rate_hz, fov_deg, max_dist_mm)
    # RMHC_SLAM uses random-mutation hill climbing to find the best pose update.
    laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
    slam = RMHC_SLAM(
        laser,
        MAP_SIZE_PIXELS,
        MAP_SIZE_METERS,
        hole_width_mm=HOLE_WIDTH_MM,
        map_quality=MAP_QUALITY,
    )
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    pss.set_status(f'connected (mode {scan_mode})')
    pss.connected.value = True

    # Keep the previous good scan so we can reuse it when a scan has too few
    # valid points (e.g. the robot is facing a large open area).
    previous_distances: Optional[list[int]] = None
    round_num = 0
    last_map_update = time.monotonic()

    try:
        for raw_angles, raw_distances in lidar_driver.scan_rounds(lidar, scan_mode):
            if pss.stop_event.is_set():
                break

            round_num += 1
            pss.rounds_seen.value = round_num

            # Skip the first few scans while the motor reaches full speed.
            if round_num <= INITIAL_ROUNDS_SKIP:
                pss.valid_points.value = 0
                pss.set_status(f'warming up {round_num}/{INITIAL_ROUNDS_SKIP}')
                continue

            # Honour pause requests from the UI.
            if pss.paused.value:
                pss.set_status('paused')
                continue

            # Resample raw measurements into fixed-size angle bins.
            scan_distances, valid = _resample_scan(raw_angles, raw_distances)
            pss.valid_points.value = valid

            # Choose which scan to feed SLAM.
            if valid >= MIN_VALID_POINTS:
                # Enough fresh data: do a full SLAM update.
                slam.update(scan_distances, scan_angles_degrees=_SCAN_ANGLES)
                previous_distances = list(scan_distances)
                note = f'live ({valid} pts)'
            elif previous_distances is not None:
                # Too few fresh readings: reuse the last good scan to keep
                # the pose estimate from drifting.
                slam.update(previous_distances, scan_angles_degrees=_SCAN_ANGLES)
                note = f'reusing previous ({valid} pts)'
            else:
                # No previous scan available yet; wait for a better scan.
                pss.set_status(f'waiting ({valid} pts)')
                continue

            # Read the updated robot pose.
            x_mm, y_mm, theta_deg = slam.getpos()
            pss.x_mm.value = x_mm
            pss.y_mm.value = y_mm
            pss.theta_deg.value = theta_deg
            pss.pose_version.value += 1

            # Copy the updated map into shared memory at a throttled rate.
            # Copying 1 MB on every scan would be expensive; once per second
            # is enough for the UI to appear responsive.
            now = time.monotonic()
            if now - last_map_update >= MAP_UPDATE_INTERVAL:
                slam.getmap(mapbytes)
                pss.shm.buf[:len(mapbytes)] = mapbytes
                pss.map_version.value += 1
                last_map_update = now

            pss.set_status(note)

    except Exception as exc:
        pss.set_error(f'SLAM process error: {exc}')
    finally:
        try:
            lidar_driver.disconnect(lidar)
        except Exception:
            pass
        pss.connected.value = False
        pss.stopped.value = True
