#!/usr/bin/env python3
"""
settings.py - All user-configurable settings for the SLAM system.

This version keeps the same filename and overall role as your current repo,
but uses a Matplotlib desktop layout that matches the screenshot style:
left panel for the occupancy map, right panel for the LiDAR point cloud.
"""

# ===========================================================================
# LIDAR hardware settings
# ===========================================================================

LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 115200

# ===========================================================================
# SLAM map settings
# ===========================================================================

# Screenshot-like defaults.
MAP_SIZE_PIXELS = 1000
MAP_SIZE_METERS = 5
MAP_QUALITY = 7
HOLE_WIDTH_MM = 100

# ===========================================================================
# Scan settings
# ===========================================================================

SCAN_SIZE = 360
SCAN_RATE_HZ = 5
DETECTION_ANGLE = 360
MAX_DISTANCE_MM = 5000

# ===========================================================================
# LIDAR mounting offset
# ===========================================================================

LIDAR_OFFSET_DEG = 0

# ===========================================================================
# Scan quality thresholds
# ===========================================================================

MIN_VALID_POINTS = 150
INITIAL_ROUNDS_SKIP = 5

# ===========================================================================
# UI and rendering settings
# ===========================================================================

# Refresh rates
UI_REFRESH_HZ = 8
MAP_UPDATE_HZ = 2.0
MAP_UPDATE_INTERVAL = 1.0 / MAP_UPDATE_HZ

# BreezySLAM uses 0 = wall, 127 = unknown, 255 = free.
UNKNOWN_BYTE = 127

# Point cloud view limits in metres, similar to the reference screenshot.
POINT_CLOUD_HALF_WIDTH_M = 2.5

# Robot drawing.
ROBOT_MARKER_SIZE = 8
HEADING_ARROW_LENGTH_M = 0.18
HEADING_ARROW_HEAD_WIDTH_M = 0.05

# Shared scan history.
POINT_CLOUD_MAX_POINTS = SCAN_SIZE
