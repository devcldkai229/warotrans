"""AMCL readiness — a fresh map->base_link TF alone does not mean the robot knows where it is.

amcl.yaml boots with set_initial_pose (0, 0, 0) so map->odom exists for Nav2 activation; TF is fresh from the
first scan and the gateway used to report LOCALIZED (and accept NAVIGATE) before anyone placed the robot.
On the real robot this requires:
  - an operator pose (/initialpose: RViz / Foxglove "2D Pose Estimate") received since the gateway started, and
  - the latest /amcl_pose covariance within limits (a diverged filter reports a wide covariance).
Simulation spawns at a known pose and keeps the TF-only behaviour (require_amcl_converged: false).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass
class AmclReadiness:
    max_xy_std_m: float = 0.5
    max_yaw_std_rad: float = 0.5
    _operator_pose: bool = False
    _xy_std: Optional[float] = None
    _yaw_std: Optional[float] = None

    def on_initial_pose(self) -> None:
        self._operator_pose = True

    def on_amcl_pose(self, covariance: Sequence[float]) -> None:
        """Row-major 6x6 (x, y, z, roll, pitch, yaw) covariance of PoseWithCovariance."""
        if len(covariance) < 36:
            return
        var_x, var_y, var_yaw = float(covariance[0]), float(covariance[7]), float(covariance[35])
        if not all(math.isfinite(v) and v >= 0.0 for v in (var_x, var_y, var_yaw)):
            self._xy_std = self._yaw_std = math.inf
            return
        self._xy_std = math.sqrt(max(var_x, var_y))
        self._yaw_std = math.sqrt(var_yaw)

    def blocking_reason(self) -> Optional[str]:
        """None when the AMCL pose may be trusted for motion; otherwise why not."""
        if not self._operator_pose:
            return "NO_OPERATOR_INITIAL_POSE"
        if self._xy_std is None or self._yaw_std is None:
            return "NO_AMCL_POSE"
        if self._xy_std > self.max_xy_std_m or self._yaw_std > self.max_yaw_std_rad:
            return f"AMCL_COVARIANCE_HIGH xy_std={self._xy_std:.3f} yaw_std={self._yaw_std:.3f}"
        return None

    @property
    def xy_std(self) -> Optional[float]:
        return self._xy_std

    @property
    def yaw_std(self) -> Optional[float]:
        return self._yaw_std
