"""Map Nav2 / action_msgs GoalStatus codes to fleet navigationStatus strings."""

from __future__ import annotations

# action_msgs/msg/GoalStatus constants (numeric) — keep local to avoid ROS import in unit tests.
STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6


class Nav2StatusTracker:
    """Tracks latest navigate_to_pose goal status for telemetry."""

    def __init__(self) -> None:
        self._status: str = "IDLE"
        self._ever_seen_goal = False

    @property
    def navigation_status(self) -> str:
        return self._status

    def update_from_goal_status(self, status_code: int) -> str:
        self._ever_seen_goal = True
        mapped = self.map_goal_status(status_code)
        self._status = mapped
        return mapped

    def mark_idle(self) -> None:
        self._status = "IDLE"

    @staticmethod
    def map_goal_status(status_code: int) -> str:
        if status_code in (STATUS_ACCEPTED, STATUS_EXECUTING):
            return "NAVIGATING"
        if status_code == STATUS_CANCELING:
            return "PAUSED"
        if status_code == STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if status_code == STATUS_ABORTED:
            return "FAILED"
        if status_code == STATUS_CANCELED:
            return "CANCELED"
        return "IDLE"
