#!/usr/bin/env python3
"""
pose_ekf.py - 3-state Extended Kalman Filter pose smoother.

State vector: [x_mm, y_mm, theta_deg]

Since there is no wheel odometry, the prediction step uses an identity
motion model (the robot could be anywhere — only process noise grows the
uncertainty). The update step fuses BreezySLAM's getpos() as a noisy
measurement on every scan.

The net effect is a weighted average between the previous smoothed pose
and the new raw BreezySLAM pose.  Noisy scan-to-scan jumps in x/y and
theta are suppressed without introducing lag in the occupancy map (the
map is still built from the raw SLAM poses internally).

Uses only numpy — no additional dependencies.
"""

from __future__ import annotations
import numpy as np


class PoseEKF:
    """Kalman filter over robot pose (x_mm, y_mm, theta_deg).

    Parameters
    ----------
    sigma_process_xy_mm : how much x/y can drift between scans (mm).
        Raise if the robot moves quickly; lower for a slower robot.
    sigma_process_th_deg : how much theta can drift between scans (deg).
    sigma_meas_xy_mm : assumed noise on BreezySLAM's x/y output (mm).
    sigma_meas_th_deg : assumed noise on BreezySLAM's theta output (deg).
        Higher = EKF trusts the filter more than the raw measurement.
    """

    def __init__(
        self,
        sigma_process_xy_mm:  float = 50.0,
        sigma_process_th_deg: float = 3.0,
        sigma_meas_xy_mm:     float = 30.0,
        sigma_meas_th_deg:    float = 8.0,
    ) -> None:
        self._x: np.ndarray | None = None          # state [x, y, θ]
        self._P  = np.diag([1e8, 1e8, 1e4])        # large initial covariance
        self._Q  = np.diag([                        # process noise
            sigma_process_xy_mm  ** 2,
            sigma_process_xy_mm  ** 2,
            sigma_process_th_deg ** 2,
        ])
        self._R  = np.diag([                        # measurement noise
            sigma_meas_xy_mm  ** 2,
            sigma_meas_xy_mm  ** 2,
            sigma_meas_th_deg ** 2,
        ])
        self._I3 = np.eye(3)

    # ------------------------------------------------------------------
    # Prediction step  (identity motion model — no odometry available)
    # ------------------------------------------------------------------

    def predict(self) -> None:
        """Grow covariance by process noise; state is unchanged."""
        if self._x is None:
            return
        # F = I  (no motion model), so P = F P Fᵀ + Q = P + Q
        self._P += self._Q

    # ------------------------------------------------------------------
    # Update step  (BreezySLAM pose as measurement)
    # ------------------------------------------------------------------

    def update(self, x_mm: float, y_mm: float, theta_deg: float) -> None:
        """Fuse a new BreezySLAM pose measurement into the filter."""
        z = np.array([x_mm, y_mm, theta_deg], dtype=np.float64)

        if self._x is None:
            # First measurement — bootstrap the state directly.
            self._x = z.copy()
            return

        # Innovation (residual), with angle wrap for theta.
        y        = z - self._x
        y[2]     = (y[2] + 180.0) % 360.0 - 180.0   # keep in (−180, +180]

        # H = I  (measurement maps directly to state)
        # S = H P Hᵀ + R = P + R
        S        = self._P + self._R                  # innovation covariance
        K        = self._P @ np.linalg.inv(S)         # Kalman gain

        self._x  = self._x + K @ y
        self._P  = (self._I3 - K) @ self._P

    # ------------------------------------------------------------------

    def get_pose(self) -> tuple[float, float, float]:
        """Return the current smoothed estimate (x_mm, y_mm, theta_deg)."""
        if self._x is None:
            return 0.0, 0.0, 0.0
        return float(self._x[0]), float(self._x[1]), float(self._x[2])