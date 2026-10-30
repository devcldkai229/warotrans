"""Motion authority (Active Mode, ADR 0009/0010) — dataclass, parser và store có persist.

Pure Python — không import ROS. Thời gian:
- ``expires_at`` là unix time (wall clock, float) vì backend gửi ISO-8601 UTC.
- LRU/ages khác dùng clock được inject để test.

AuthorityStore chỉ quyết định *chấp nhận hay từ chối* một authority theo (epoch, seq, expiry,
duplicate). Nó KHÔNG quyết định robot được chạy: sau restart ``motion_resumable=False`` —
authority cũ chỉ để báo cáo (authority_state), muốn chạy phải có seq cao hơn từ backend.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Optional

from warotrans_fleet.command_contract import (
    REASON_AUTHORITY_DUPLICATE,
    REASON_AUTHORITY_EXPIRED,
    REASON_AUTHORITY_REQUIRED,
    REASON_AUTHORITY_STALE_EPOCH,
    REASON_AUTHORITY_STALE_SEQ,
    REASON_INVALID_PAYLOAD,
    REASON_PATH_HASH_MISMATCH,
    CommandRejected,
)
from warotrans_fleet.route_grant import compute_path_hash

__all__ = [
    "Authority",
    "AuthorityDecision",
    "AuthorityStore",
    "DEFAULT_STATE_FILENAME",
    "LRU_CAPACITY",
    "compute_path_hash",
    "default_state_path",
    "parse_authority",
    "parse_edge_ids",
    "parse_expires_at",
    "parse_speed_limits",
]

LRU_CAPACITY = 256
DEFAULT_STATE_FILENAME = "warotrans_authority_state.json"
_STATE_VERSION = 1


@dataclass(frozen=True)
class Authority:
    authority_id: str
    epoch: int
    seq: int
    graph_hash: str
    path_hash: str
    segment_edge_ids: tuple[int, ...]
    map_version_id: Optional[str] = None
    map_version_code: Optional[str] = None
    expires_at: Optional[float] = None  # unix time; None = không hết hạn
    speed_limits: Mapping[str, float] = field(default_factory=dict)
    route_id: Optional[str] = None
    route_version: int = 0
    goal_safe_stop_node_id: Optional[int] = None
    final_goal: Optional[tuple[float, float, float]] = None  # (x, y, yaw) trong frame map
    command_id: Optional[str] = None

    def is_expired(self, now: float) -> bool:
        return self.expires_at is not None and now >= self.expires_at

    def expected_path_hash(self) -> str:
        return compute_path_hash(self.graph_hash, self.segment_edge_ids)


@dataclass(frozen=True)
class AuthorityDecision:
    accepted: bool
    reason_code: Optional[str] = None  # None khi accepted

    def __bool__(self) -> bool:  # pragma: no cover - tiện dùng `if store.accept(...)`
        return self.accepted


# ----------------------------------------------------------------------------- parsing


def parse_expires_at(value: Any) -> Optional[float]:
    """ISO-8601 string (có hoặc không 'Z') hoặc số unix → unix float. None → None."""
    if value is None:
        return None
    if isinstance(value, bool):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'expiresAt' must be ISO-8601 string")
    if isinstance(value, (int, float)):
        number = float(value)
        if not math.isfinite(number):
            raise CommandRejected(REASON_INVALID_PAYLOAD, "'expiresAt' must be finite")
        return number
    if not isinstance(value, str) or not value.strip():
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'expiresAt' must be ISO-8601 string")
    text = value.strip()
    if text.endswith(("Z", "z")):
        text = text[:-1] + "+00:00"
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError as exc:
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'expiresAt' invalid: {exc}") from exc
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.timestamp()


def _int_field(src: Mapping[str, Any], key: str, *, required: bool = True, default: int = 0) -> int:
    if key not in src or src[key] is None:
        if required:
            raise CommandRejected(REASON_INVALID_PAYLOAD, f"'authority.{key}' is required")
        return default
    value = src[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value:
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'authority.{key}' must be an integer")
    return int(value)


def _str_field(src: Mapping[str, Any], key: str, *, required: bool = True) -> Optional[str]:
    value = src.get(key)
    if value is None or value == "":
        if required:
            raise CommandRejected(REASON_INVALID_PAYLOAD, f"'authority.{key}' is required")
        return None
    if not isinstance(value, str):
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'authority.{key}' must be a string")
    return value.strip()


def parse_edge_ids(raw: Any, *, label: str = "authority.segmentEdgeIds") -> tuple[int, ...]:
    if not isinstance(raw, (list, tuple)):
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'{label}' must be an array")
    ids: list[int] = []
    for item in raw:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or int(item) != item:
            raise CommandRejected(REASON_INVALID_PAYLOAD, f"'{label}' must contain integers")
        ids.append(int(item))
    return tuple(ids)


def parse_speed_limits(raw: Any) -> dict[str, float]:
    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'authority.speedLimits' must be an object")
    limits: dict[str, float] = {}
    for key, value in raw.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise CommandRejected(REASON_INVALID_PAYLOAD, f"'authority.speedLimits.{key}' must be a finite number")
        limits[str(key)] = float(value)
    return limits


def parse_final_goal(raw: Any) -> Optional[tuple[float, float, float]]:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'authority.finalGoal' must be an object")
    values: list[float] = []
    for key in ("x", "y", "yaw"):
        value = raw.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise CommandRejected(REASON_INVALID_PAYLOAD, f"'authority.finalGoal.{key}' must be a finite number")
        values.append(float(value))
    return (values[0], values[1], values[2])


def parse_authority(
    payload: Mapping[str, Any],
    *,
    command_id: Optional[str] = None,
    verify_path_hash: bool = True,
) -> Authority:
    """Parse ``payload.authority``. Raises CommandRejected.

    - thiếu authority → AUTHORITY_REQUIRED
    - field sai kiểu → INVALID_PAYLOAD
    - pathHash không khớp ``sha256(graphHash|edgeIds)`` → PATH_HASH_MISMATCH
    """
    src = payload.get("authority") if isinstance(payload, Mapping) else None
    if src is None:
        raise CommandRejected(REASON_AUTHORITY_REQUIRED, "payload.authority is required in active traffic mode")
    if not isinstance(src, Mapping):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.authority' must be an object")

    graph_hash = _str_field(src, "graphHash")
    path_hash = _str_field(src, "pathHash")
    assert graph_hash is not None and path_hash is not None
    edge_ids = parse_edge_ids(src.get("segmentEdgeIds"))

    goal_node = src.get("goalSafeStopNodeId")
    if goal_node is not None:
        goal_node = _int_field(src, "goalSafeStopNodeId")

    authority = Authority(
        authority_id=_str_field(src, "authorityId") or "",
        epoch=_int_field(src, "epoch"),
        seq=_int_field(src, "seq"),
        graph_hash=graph_hash,
        path_hash=path_hash.lower(),
        segment_edge_ids=edge_ids,
        map_version_id=_str_field(src, "mapVersionId", required=False),
        map_version_code=_str_field(src, "mapVersionCode", required=False),
        expires_at=parse_expires_at(src.get("expiresAt")),
        speed_limits=parse_speed_limits(src.get("speedLimits")),
        route_id=_str_field(src, "routeId", required=False),
        route_version=_int_field(src, "routeVersion", required=False, default=0),
        goal_safe_stop_node_id=goal_node,
        final_goal=parse_final_goal(src.get("finalGoal")),
        command_id=command_id,
    )
    if verify_path_hash and authority.expected_path_hash() != authority.path_hash:
        raise CommandRejected(
            REASON_PATH_HASH_MISMATCH,
            f"authority.pathHash {authority.path_hash[:12]}… != sha256(graphHash|edgeIds)",
        )
    return authority


# ----------------------------------------------------------------------------- persistence helpers


def default_state_path() -> str:
    return str(Path(tempfile.gettempdir()) / DEFAULT_STATE_FILENAME)


def _authority_to_json(a: Authority) -> dict[str, Any]:
    return {
        "authorityId": a.authority_id,
        "epoch": a.epoch,
        "seq": a.seq,
        "graphHash": a.graph_hash,
        "pathHash": a.path_hash,
        "segmentEdgeIds": list(a.segment_edge_ids),
        "mapVersionId": a.map_version_id,
        "mapVersionCode": a.map_version_code,
        "expiresAt": a.expires_at,
        "speedLimits": dict(a.speed_limits),
        "routeId": a.route_id,
        "routeVersion": a.route_version,
        "goalSafeStopNodeId": a.goal_safe_stop_node_id,
        "finalGoal": None
        if a.final_goal is None
        else {"x": a.final_goal[0], "y": a.final_goal[1], "yaw": a.final_goal[2]},
        "commandId": a.command_id,
    }


def _authority_from_json(raw: Mapping[str, Any]) -> Authority:
    goal = raw.get("finalGoal")
    return Authority(
        authority_id=str(raw["authorityId"]),
        epoch=int(raw["epoch"]),
        seq=int(raw["seq"]),
        graph_hash=str(raw["graphHash"]),
        path_hash=str(raw["pathHash"]),
        segment_edge_ids=tuple(int(e) for e in raw.get("segmentEdgeIds", [])),
        map_version_id=raw.get("mapVersionId"),
        map_version_code=raw.get("mapVersionCode"),
        expires_at=None if raw.get("expiresAt") is None else float(raw["expiresAt"]),
        speed_limits={str(k): float(v) for k, v in (raw.get("speedLimits") or {}).items()},
        route_id=raw.get("routeId"),
        route_version=int(raw.get("routeVersion") or 0),
        goal_safe_stop_node_id=None if raw.get("goalSafeStopNodeId") is None else int(raw["goalSafeStopNodeId"]),
        final_goal=None
        if not isinstance(goal, Mapping)
        else (float(goal["x"]), float(goal["y"]), float(goal["yaw"])),
        command_id=raw.get("commandId"),
    )


# ----------------------------------------------------------------------------- store


class AuthorityStore:
    """Giữ (epoch, seq) cao nhất đã chấp nhận, authority đang active và LRU chống duplicate.

    ``clear_motion_on_restart=True`` (mặc định, an toàn): sau khi load từ file, authority cũ vẫn
    được giữ để báo cáo nhưng ``motion_resumable`` = False. Robot không tự chạy tiếp.
    """

    def __init__(
        self,
        path: Optional[str] = None,
        *,
        clock: Callable[[], float] = time.time,
        lru_capacity: int = LRU_CAPACITY,
        clear_motion_on_restart: bool = True,
        persist: bool = True,
    ) -> None:
        if lru_capacity < 1:
            raise ValueError("lru_capacity must be >= 1")
        self._path = path or default_state_path()
        self._clock = clock
        self._capacity = lru_capacity
        self._clear_motion_on_restart = clear_motion_on_restart
        self._persist_enabled = persist
        self._last_epoch = 0
        self._last_seq = -1
        self._active: Optional[Authority] = None
        self._motion_resumable = True
        self._seen: "OrderedDict[tuple[str, str], None]" = OrderedDict()
        self._load()

    # -- read-only view

    @property
    def path(self) -> str:
        return self._path

    @property
    def active(self) -> Optional[Authority]:
        return self._active

    @property
    def last_epoch(self) -> int:
        return self._last_epoch

    @property
    def last_seq(self) -> int:
        return self._last_seq

    @property
    def motion_resumable(self) -> bool:
        """False sau restart: authority chỉ để báo cáo, không được dùng để chạy tiếp."""
        return self._motion_resumable and self._active is not None

    @property
    def seen_count(self) -> int:
        return len(self._seen)

    # -- mutate

    def accept(
        self,
        authority: Authority,
        *,
        command_id: Optional[str] = None,
        message_id: Optional[str] = None,
        now: Optional[float] = None,
    ) -> AuthorityDecision:
        """Chấp nhận authority mới. Thứ tự kiểm tra: DUPLICATE → EXPIRED → STALE_EPOCH → STALE_SEQ."""
        key = self._dedupe_key(command_id or authority.command_id, message_id)
        if key is not None and key in self._seen:
            self._seen.move_to_end(key)
            return AuthorityDecision(False, REASON_AUTHORITY_DUPLICATE)

        wall = self._clock() if now is None else now
        if authority.is_expired(wall):
            return AuthorityDecision(False, REASON_AUTHORITY_EXPIRED)

        if authority.epoch < self._last_epoch:
            return AuthorityDecision(False, REASON_AUTHORITY_STALE_EPOCH)
        if authority.epoch == self._last_epoch and authority.seq <= self._last_seq:
            return AuthorityDecision(False, REASON_AUTHORITY_STALE_SEQ)

        self._last_epoch = authority.epoch
        self._last_seq = authority.seq
        self._active = authority
        self._motion_resumable = True
        if key is not None:
            self._seen[key] = None
            while len(self._seen) > self._capacity:
                self._seen.popitem(last=False)
        self._save()
        return AuthorityDecision(True, None)

    def clear_active(self) -> None:
        """Bỏ authority đang giữ (command kết thúc/hủy). (epoch, seq) cao nhất vẫn được giữ."""
        self._active = None
        self._motion_resumable = True
        self._save()

    def replace_active(self, authority: Authority) -> None:
        """Cập nhật nội dung authority đang active mà không đổi (epoch, seq) — dùng nội bộ."""
        self._active = authority
        self._save()

    def report(self, *, state: str = "IDLE") -> dict[str, Any]:
        """Dữ liệu cho AuthorityStateMessage (sync khi boot/reconnect). Không bao giờ ngụ ý 'được chạy'."""
        active = self._active
        return {
            "activeAuthorityId": active.authority_id if active else None,
            "lastEpoch": self._last_epoch,
            "lastSeq": self._last_seq,
            "state": state,
            "activeCommandId": active.command_id if active else None,
            "grantedEdgeIds": list(active.segment_edge_ids) if active else [],
            "motionResumable": self.motion_resumable,
        }

    # -- internals

    @staticmethod
    def _dedupe_key(command_id: Optional[str], message_id: Optional[str]) -> Optional[tuple[str, str]]:
        if not command_id and not message_id:
            return None
        return (command_id or "", message_id or "")

    def _save(self) -> None:
        if not self._persist_enabled:
            return
        data = {
            "version": _STATE_VERSION,
            "lastEpoch": self._last_epoch,
            "lastSeq": self._last_seq,
            "active": None if self._active is None else _authority_to_json(self._active),
            "seen": [list(k) for k in self._seen],
        }
        directory = os.path.dirname(os.path.abspath(self._path))
        try:
            os.makedirs(directory, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix=".authority_", suffix=".tmp", dir=directory)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as handle:
                    json.dump(data, handle)
                os.replace(tmp_path, self._path)
            except BaseException:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
        except OSError:
            # Không persist được thì vẫn chạy bằng RAM; restart sau đó sẽ mất seq → backend sync lại.
            pass

    def _load(self) -> None:
        if not self._persist_enabled:
            return
        try:
            with open(self._path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            if not isinstance(data, Mapping) or data.get("version") != _STATE_VERSION:
                return
            last_epoch = int(data.get("lastEpoch", 0))
            last_seq = int(data.get("lastSeq", -1))
            active_raw = data.get("active")
            active = _authority_from_json(active_raw) if isinstance(active_raw, Mapping) else None
            seen: "OrderedDict[tuple[str, str], None]" = OrderedDict()
            for item in data.get("seen", []):
                if isinstance(item, (list, tuple)) and len(item) == 2:
                    seen[(str(item[0]), str(item[1]))] = None
        except (OSError, ValueError, KeyError, TypeError):
            return  # file hỏng/không có: bắt đầu sạch, backend sync lại
        self._last_epoch = last_epoch
        self._last_seq = last_seq
        self._active = active
        self._seen = seen
        while len(self._seen) > self._capacity:
            self._seen.popitem(last=False)
        if active is not None and self._clear_motion_on_restart:
            self._motion_resumable = False

