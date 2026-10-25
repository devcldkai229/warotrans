"""SafetySupervisor — logic 20 Hz quyết định ``authority_ok`` (Active Mode).

Pure Python: gọi ``tick()`` từ timer ROS 20 Hz (hoặc trực tiếp từ test). Mọi input là provider.

Hai loại phản ứng:
- **Operational pause** (tạm thời, tự hết): pose/scan quá cũ, mất localization, edge kế tiếp chưa
  được cấp quyền. ``authority_ok=False`` → gate chặn /cmd_vel_safe; robot đứng yên nhưng
  TrafficClient vẫn EXECUTING. Hết điều kiện → ``authority_ok=True`` lại. Nếu pause kéo dài quá
  ``pause_fault_timeout_sec`` thì leo thang thành fault.
- **Emergency fault** (latched): authority mất/hết hạn, ra khỏi corridor, watchdog MQTT authority,
  robot không tiến triển. Gọi ``on_fault(code, detail)`` (→ TrafficClient.fault) và latch
  ``authority_ok=False`` cho tới khi client rời FAULTED rồi nhận lại motion mới.

Supervisor chỉ hạ/giữ ``authority_ok``. "Đã dừng" KHÔNG BAO GIỜ được báo như "release/hoàn thành":
không có API nào ở đây phát tín hiệu giải phóng hay SUCCEEDED.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from warotrans_fleet.authority_store import Authority
from warotrans_fleet.command_contract import (
    ERROR_AUTHORITY_EXPIRED,
    ERROR_AUTHORITY_LOST,
    ERROR_CONTROLLER_FAILED,
    ERROR_CORRIDOR_VIOLATION,
    ERROR_LOCALIZATION_LOST,
    ERROR_SAFETY_STOP,
)
from warotrans_fleet.route_grant import GraphEdge, outside_corridor

TICK_HZ = 20.0

STATUS_IDLE = "IDLE"  # không kỳ vọng motion
STATUS_OK = "OK"
STATUS_PAUSED = "PAUSED"
STATUS_FAULT = "FAULT"

PAUSE_POSE_STALE = "POSE_STALE"
PAUSE_SCAN_STALE = "SCAN_STALE"
PAUSE_LOCALIZATION_LOST = "LOCALIZATION_LOST"
PAUSE_NEXT_EDGE_NOT_GRANTED = "NEXT_EDGE_NOT_GRANTED"


@dataclass(frozen=True)
class SafetyConfig:
    pose_max_age_sec: float = 0.5
    scan_max_age_sec: float = 0.5
    corridor_buffer_m: float = 0.35
    authority_loss_timeout_sec: float = 3.0
    progress_timeout_sec: float = 8.0  # 0 = tắt kiểm tra progress
    progress_epsilon: float = 0.02  # tiến triển tối thiểu (đơn vị của progress_provider, mét)
    pause_fault_timeout_sec: float = 5.0  # 0 = pause không bao giờ leo thang


@dataclass(frozen=True)
class SupervisorStatus:
    authority_ok: bool
    state: str  # STATUS_*
    reason: Optional[str] = None
    fault_code: Optional[str] = None


class SafetySupervisor:
    def __init__(
        self,
        *,
        config: SafetyConfig = SafetyConfig(),
        authority_provider: Callable[[], Optional[Authority]],
        motion_expected: Callable[[], bool],
        pose_provider: Callable[[], Optional[tuple[float, float]]] = lambda: None,
        pose_age_provider: Callable[[], Optional[float]] = lambda: None,
        scan_age_provider: Callable[[], Optional[float]] = lambda: None,
        localization_ok_provider: Callable[[], bool] = lambda: False,
        progress_provider: Callable[[], Optional[float]] = lambda: None,
        last_authority_at_provider: Callable[[], Optional[float]] = lambda: None,
        next_edge_provider: Callable[[], Optional[int]] = lambda: None,
        edges: Sequence[GraphEdge] = (),
        on_fault: Callable[[str, str], None] = lambda _code, _detail: None,
        on_authority_ok: Optional[Callable[[bool], None]] = None,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        self._config = config
        self._authority_provider = authority_provider
        self._motion_expected = motion_expected
        self._pose_provider = pose_provider
        self._pose_age = pose_age_provider
        self._scan_age = scan_age_provider
        self._localization_ok = localization_ok_provider
        self._progress = progress_provider
        self._last_authority_at = last_authority_at_provider
        self._next_edge = next_edge_provider
        self._edges = list(edges)
        self._on_fault = on_fault
        self._on_authority_ok = on_authority_ok
        self._clock = clock
        self._wall_clock = wall_clock

        self._authority_ok = False
        self._latched_code: Optional[str] = None
        self._latch_saw_idle = False
        self._pause_since: Optional[float] = None
        self._progress_best: Optional[float] = None
        self._progress_at: Optional[float] = None
        self._last_status = SupervisorStatus(False, STATUS_IDLE)

    # ------------------------------------------------------------------ view

    @property
    def authority_ok(self) -> bool:
        return self._authority_ok

    @property
    def last_status(self) -> SupervisorStatus:
        return self._last_status

    @property
    def latched_fault(self) -> Optional[str]:
        return self._latched_code

    def reset(self) -> None:
        """Xóa latch (chỉ dùng khi gateway biết client đã rời FAULTED)."""
        self._latched_code = None
        self._latch_saw_idle = False
        self._clear_trackers()

    # ------------------------------------------------------------------ tick

    def tick(self) -> SupervisorStatus:
        status = self._evaluate()
        self._authority_ok = status.authority_ok
        self._last_status = status
        if self._on_authority_ok is not None:
            self._on_authority_ok(self._authority_ok)  # publish đều 20 Hz để gate đo age
        return status

    def _evaluate(self) -> SupervisorStatus:
        motion = bool(self._motion_expected())

        if self._latched_code is not None:
            if not motion:
                self._latch_saw_idle = True
            elif self._latch_saw_idle:
                self.reset()  # motion mới hợp lệ sau khi client đã rời FAULTED
            if self._latched_code is not None:
                return SupervisorStatus(False, STATUS_FAULT, "latched", self._latched_code)

        if not motion:
            self._clear_trackers()
            return SupervisorStatus(False, STATUS_IDLE)

        now = self._clock()
        authority = self._authority_provider()

        if authority is None:
            return self._fault(ERROR_AUTHORITY_LOST, "motion expected but no active authority")
        if authority.is_expired(self._wall_clock()):
            return self._fault(ERROR_AUTHORITY_EXPIRED, "authority expired")

        last = self._last_authority_at()
        if last is not None and now - last > self._config.authority_loss_timeout_sec:
            return self._fault(
                ERROR_AUTHORITY_LOST,
                f"no authority message for {now - last:.1f}s (> {self._config.authority_loss_timeout_sec}s)",
            )

        pose_age = self._pose_age()
        pose = self._pose_provider()
        pose_fresh = pose is not None and pose_age is not None and pose_age <= self._config.pose_max_age_sec
        if pose_fresh and self._edges and outside_corridor(
            pose[0], pose[1], self._edges, authority.segment_edge_ids, buffer_m=self._config.corridor_buffer_m
        ):
            return self._fault(ERROR_CORRIDOR_VIOLATION, "pose outside authorized corridor")

        reasons: list[str] = []
        if not pose_fresh:
            reasons.append(PAUSE_POSE_STALE)
        scan_age = self._scan_age()
        if scan_age is None or scan_age > self._config.scan_max_age_sec:
            reasons.append(PAUSE_SCAN_STALE)
        if not self._localization_ok():
            reasons.append(PAUSE_LOCALIZATION_LOST)
        next_edge = self._next_edge()
        if next_edge is not None and next_edge not in authority.segment_edge_ids:
            reasons.append(PAUSE_NEXT_EDGE_NOT_GRANTED)

        if reasons:
            if self._pause_since is None:
                self._pause_since = now
            self._progress_at = None  # không tính stall khi đang pause
            limit = self._config.pause_fault_timeout_sec
            if limit > 0 and now - self._pause_since >= limit:
                code = ERROR_LOCALIZATION_LOST if PAUSE_LOCALIZATION_LOST in reasons else ERROR_SAFETY_STOP
                return self._fault(code, f"operational pause too long: {','.join(reasons)}")
            return SupervisorStatus(False, STATUS_PAUSED, ",".join(reasons))

        self._pause_since = None

        stall = self._check_progress(now)
        if stall is not None:
            return stall
        return SupervisorStatus(True, STATUS_OK)

    # ------------------------------------------------------------------ helpers

    def _check_progress(self, now: float) -> Optional[SupervisorStatus]:
        timeout = self._config.progress_timeout_sec
        if timeout <= 0:
            return None
        value = self._progress()
        if value is None:
            return None
        if self._progress_best is None or self._progress_at is None:
            if self._progress_best is None:
                self._progress_best = value
            self._progress_at = now
            return None
        if value - self._progress_best >= self._config.progress_epsilon:
            self._progress_best = value
            self._progress_at = now
            return None
        if now - self._progress_at >= timeout:
            return self._fault(ERROR_CONTROLLER_FAILED, f"no progress for {now - self._progress_at:.1f}s")
        return None

    def _clear_trackers(self) -> None:
        self._pause_since = None
        self._progress_best = None
        self._progress_at = None

    def _fault(self, code: str, detail: str) -> SupervisorStatus:
        self._latched_code = code
        self._latch_saw_idle = False
        self._clear_trackers()
        self._on_fault(code, detail)
        return SupervisorStatus(False, STATUS_FAULT, detail, code)
