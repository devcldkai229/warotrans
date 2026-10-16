"""navigationStatus cho telemetry của command do fleet điều khiển.

IDLE → NAVIGATING → SUCCEEDED | FAILED | CANCELED → (sau hold) IDLE

Giữ trạng thái kết thúc `terminal_hold_sec` để telemetry kịp phản ánh transition,
sau đó về IDLE (robot đã sẵn sàng nhận MOVE mới ngay khi có RESULT, không phụ thuộc hold).
PAUSED có trong contract nhưng V1 chưa có lệnh pause nên không dùng.
"""

from __future__ import annotations

from typing import Callable, Optional

IDLE = "IDLE"
NAVIGATING = "NAVIGATING"
PAUSED = "PAUSED"
SUCCEEDED = "SUCCEEDED"
FAILED = "FAILED"
CANCELED = "CANCELED"

TERMINAL_STATES = (SUCCEEDED, FAILED, CANCELED)


class NavigationStateMachine:
    def __init__(
        self,
        *,
        clock: Callable[[], float],
        terminal_hold_sec: float = 3.0,
        on_change: Optional[Callable[[str, str], None]] = None,
    ) -> None:
        self._clock = clock
        self._hold = max(0.0, float(terminal_hold_sec))
        self._on_change = on_change
        self._status = IDLE
        self._terminal_since: Optional[float] = None

    @property
    def status(self) -> str:
        return self._status

    def start(self) -> None:
        self._set(NAVIGATING)
        self._terminal_since = None

    def finish(self, terminal: str) -> None:
        if terminal not in TERMINAL_STATES:
            raise ValueError(f"not a terminal navigation state: {terminal}")
        self._set(terminal)
        self._terminal_since = self._clock()

    def tick(self) -> None:
        if self._terminal_since is None:
            return
        if self._clock() - self._terminal_since >= self._hold:
            self._terminal_since = None
            self._set(IDLE)

    def _set(self, new: str) -> None:
        old, self._status = self._status, new
        if old != new and self._on_change is not None:
            self._on_change(old, new)
