"""Stage-3 semantic_active traffic client: propose the Nav2 plan, move only inside what the Backend reserved.

The robot may move only while ALL of these hold (checked every tick, fail closed):

* a grant for the current backend NAVIGATE exists and was not revoked;
* its epoch is not older than the newest coordinator epoch announced (retained ``traffic/epoch``) — after a backend
  restart the old coordinator's reservations are gone;
* an EXTEND/RENEW arrived within ``authority_loss_timeout_sec`` (robot monotonic clock: independent of clock skew)
  and the backend ``expiresAt`` has not passed;
* the MQTT link is up;
* the footprint at the robot's real pose (any heading, plus ``stop_margin_m``) lies inside the granted cells/zones —
  and, while a newer plan revision waits for its grant, inside the part of the old grant the new path still uses.

There is no time-based grace across a replan: a path the backend has not granted never widens where the robot may
go. The cell/zone geometry is identical to the backend's ``SemanticReservation`` (shared test vectors).
"""

from __future__ import annotations

import hashlib
import json
import math
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Iterable, Optional, Sequence

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class SemanticGeometry:
    """Geometry the backend computed a grant with (``authority.geometry``)."""

    cell_size: float
    horizon: float
    sample_step: float
    reserve_radius: float
    footprint_radius: float

    @staticmethod
    def from_payload(raw: Any) -> Optional["SemanticGeometry"]:
        if not isinstance(raw, dict):
            return None
        try:
            geom = SemanticGeometry(
                cell_size=float(raw["cellSizeMeters"]),
                horizon=float(raw["horizonMeters"]),
                sample_step=float(raw["sampleStepMeters"]),
                reserve_radius=float(raw["reserveRadiusMeters"]),
                footprint_radius=float(raw["footprintRadiusMeters"]),
            )
        except (KeyError, TypeError, ValueError):
            return None
        if min(geom.cell_size, geom.horizon, geom.sample_step, geom.reserve_radius, geom.footprint_radius) <= 0:
            return None
        return geom


@dataclass
class SemanticGrant:
    authority_id: str
    epoch: int
    seq: int
    fingerprint: str
    plan_revision: int
    plan_hash: str
    command_id: str
    resource_ids: frozenset[str]
    geometry: SemanticGeometry
    expires_at: float  # wall-clock epoch seconds (backend clock)
    last_heard_mono: float


@dataclass
class SemanticTrafficClient:
    robot_code: str
    topic_prefix: str
    fingerprint: str
    publish_json: Callable[[str, dict[str, Any]], None]
    set_authority_ok: Callable[[bool], None]
    zones: list[dict[str, Any]] = field(default_factory=list)
    now: Callable[[], float] = time.time
    monotonic: Callable[[], float] = time.monotonic
    authority_loss_timeout_sec: float = 3.0
    # Extra radius checked around the footprint: distance the robot covers while the gate closes and it brakes.
    stop_margin_m: float = 0.05
    # Margin used before a resource is reported left (backend re-checks with its own localization margin).
    exit_margin_m: float = 0.05
    propose_retry_sec: float = 2.0

    plan_revision: int = 0
    plan_hash: str = ""
    plan_poses: list[Pose] = field(default_factory=list)
    command_id: Optional[str] = None
    grant: Optional[SemanticGrant] = None
    known_epoch: int = 0
    link_up: bool = True
    last_block_reason: str = "NO_GRANT"
    entered_resources: set[str] = field(default_factory=set)
    # Reported RESOURCES_CLEARED since the last grant: the backend may already have given them to another robot, even
    # though the grant still lists them. Only moving backwards (recovery) or a replan can bring the robot back to one.
    released_resources: set[str] = field(default_factory=set)
    _progress_seq: int = 0
    _last_propose_mono: float = -1e9
    _last_blocked_mono: float = -1e9
    _last_state_report_mono: float = -1e9
    _was_localized: bool = False
    _allowed_cache: Optional[tuple[tuple[int, str, str], frozenset[str]]] = None

    # ------------------------------------------------------------------ downlink: plan / command lifecycle

    def on_plan(
        self,
        poses: Sequence[Pose],
        *,
        goal: Optional[Pose] = None,
        command_id: Optional[str] = None,
        job_step_id: Optional[str] = None,
        job_id: Optional[str] = None,
    ) -> None:
        """Nav2 published a plan. Proposed only for a backend NAVIGATE; a new path bumps planRevision."""
        if len(poses) < 2:
            return
        if not command_id:
            # Only motion the backend commanded is ever authorized (no grant for RViz / CLI goals).
            self.command_id = None
            self.plan_poses = []
            return
        if command_id != self.command_id:
            self.command_id = command_id
            self.plan_hash = ""
            self._drop_grant("NEW_COMMAND")
        path_hash = plan_hash(poses)
        if path_hash == self.plan_hash:
            # Same path: re-propose only on a cooldown while no valid grant covers it (lost uplink / queued).
            if self.grant is not None and self.grant.plan_revision == self.plan_revision:
                return
            if self.monotonic() - self._last_propose_mono < self.propose_retry_sec:
                return
        else:
            self.plan_revision += 1
            self.plan_hash = path_hash
            self.plan_poses = [tuple(map(float, p)) for p in poses]  # type: ignore[misc]
            self._allowed_cache = None
        self._last_propose_mono = self.monotonic()
        payload = {
            "schemaVersion": 1,
            "messageId": str(uuid.uuid4()),
            "robotCode": self.robot_code,
            "semanticMapFingerprint": self.fingerprint,
            "planRevision": self.plan_revision,
            "planHash": self.plan_hash,
            "commandId": command_id,
            "jobStepId": job_step_id,
            "jobId": job_id,
            "poses": [{"x": p[0], "y": p[1], "yaw": p[2]} for p in poses],
            "goal": None if goal is None else {"x": goal[0], "y": goal[1], "yaw": goal[2]},
            "sentAt": _utc_now_iso(),
        }
        self.publish_json(f"{self.topic_prefix.rstrip('/')}/robots/{self.robot_code}/traffic_plan", payload)

    def on_command_finished(self, command_id: Optional[str] = None) -> None:
        """The NAVIGATE ended (result / cancel): its grant authorizes nothing any more."""
        if command_id is None or command_id == self.command_id:
            self.command_id = None
            self.plan_poses = []
            self.plan_hash = ""
            self._drop_grant("COMMAND_FINISHED")

    # ------------------------------------------------------------------ downlink: authority

    def on_authority_payload(self, authority: dict[str, Any], *, target_command_id: Optional[str] = None) -> bool:
        """EXTEND_AUTHORITY for the current plan. Returns True when accepted (motion still checked per tick)."""
        reason = self._reject_reason(authority, target_command_id)
        if reason is not None:
            self.last_block_reason = reason
            return False
        geometry = SemanticGeometry.from_payload(authority.get("geometry"))
        assert geometry is not None  # checked in _reject_reason
        self.grant = SemanticGrant(
            authority_id=str(authority.get("authorityId")),
            epoch=int(authority.get("epoch") or 0),
            seq=int(authority.get("seq") or 0),
            fingerprint=str(authority.get("semanticMapFingerprint") or ""),
            plan_revision=int(authority.get("planRevision") or 0),
            plan_hash=str(authority.get("planHash") or ""),
            command_id=str(target_command_id or self.command_id or ""),
            resource_ids=frozenset(str(x) for x in (authority.get("resourceIds") or [])),
            geometry=geometry,
            expires_at=_parse_expires_at(authority.get("expiresAt")),
            last_heard_mono=self.monotonic(),
        )
        self.known_epoch = max(self.known_epoch, self.grant.epoch)
        self.entered_resources &= set(self.grant.resource_ids)
        # This grant is the backend's current word: what it lists again is held again.
        self.released_resources -= set(self.grant.resource_ids)
        self._progress_seq = max(self._progress_seq, self.grant.seq)
        self._allowed_cache = None
        return True

    def _reject_reason(self, authority: dict[str, Any], target_command_id: Optional[str]) -> Optional[str]:
        fp = str(authority.get("semanticMapFingerprint") or "")
        if not fp or fp != self.fingerprint:
            return "FINGERPRINT_MISMATCH"
        if not self.command_id or (target_command_id and target_command_id != self.command_id):
            return "NOT_CURRENT_COMMAND"
        rev = int(authority.get("planRevision") or 0)
        if rev <= 0 or rev != self.plan_revision:
            return "STALE_PLAN_REVISION"
        if str(authority.get("planHash") or "") != self.plan_hash:
            return "PLAN_HASH_MISMATCH"
        epoch = int(authority.get("epoch") or 0)
        if epoch < self.known_epoch:
            return "STALE_EPOCH"
        authority_id = str(authority.get("authorityId") or "")
        if not authority_id:
            return "NO_AUTHORITY_ID"
        seq = int(authority.get("seq") or 0)
        g = self.grant
        if g is not None and (epoch < g.epoch or (authority_id == g.authority_id and seq <= g.seq)):
            return "DUPLICATE_OR_OLDER"
        if SemanticGeometry.from_payload(authority.get("geometry")) is None:
            return "NO_GEOMETRY"
        if _parse_expires_at(authority.get("expiresAt")) <= self.now():
            return "EXPIRED_ON_ARRIVAL"
        return None

    def on_renew(self, payload: dict[str, Any]) -> bool:
        g = self.grant
        if g is None:
            return False
        if str(payload.get("authorityId") or "") != g.authority_id or int(payload.get("epoch") or 0) != g.epoch:
            return False
        seq = int(payload.get("seq") or 0)
        if seq <= g.seq:
            return False  # duplicate / reordered renew
        expires = _parse_expires_at(payload.get("expiresAt"))
        if expires <= self.now():
            return False
        g.seq = seq
        g.expires_at = expires
        g.last_heard_mono = self.monotonic()
        return True

    def on_revoke(self, payload: Optional[dict[str, Any]] = None) -> None:
        """REVOKE_AUTHORITY stops the robot now. A revoke naming another (older) authority is ignored."""
        authority_id = str((payload or {}).get("authorityId") or "")
        if self.grant is not None and authority_id and authority_id != self.grant.authority_id:
            return
        self._drop_grant("REVOKED")
        self._set_ok(False)

    def handle_authority_command(self, body: dict[str, Any]) -> Optional[dict[str, Any]]:
        """EXTEND / RENEW / REVOKE_AUTHORITY envelope → apply it and return the command_ack to publish.

        None for any other command type (NAVIGATE / CANCEL go to the command router). The ACK is terminal for the
        backend; a rejected one carries the reason (audit of refused grants).
        """
        cmd_type = str(body.get("type") or "").upper()
        if cmd_type not in ("EXTEND_AUTHORITY", "RENEW_AUTHORITY", "REVOKE_AUTHORITY"):
            return None
        payload = body.get("payload") or {}
        if isinstance(payload, str):
            try:
                payload = json.loads(payload)
            except json.JSONDecodeError:
                payload = {}
        if not isinstance(payload, dict):
            payload = {}
        reason: Optional[str] = None
        if cmd_type == "EXTEND_AUTHORITY":
            target = str(payload.get("targetCommandId") or "") or None
            if not self.on_authority_payload(payload, target_command_id=target):
                reason = self.last_block_reason
        elif cmd_type == "RENEW_AUTHORITY":
            if not self.on_renew(payload):
                reason = "RENEW_NOT_APPLIED"
        else:
            self.on_revoke(payload)
        return {
            "schemaVersion": 1,
            "messageId": str(uuid.uuid4()),
            "commandId": body.get("commandId"),
            "robotCode": self.robot_code,
            "accepted": reason is None,
            "reasonCode": reason,
            "reasonMessage": None,
            "jobAssignmentId": body.get("jobAssignmentId"),
            "jobStepId": body.get("jobStepId"),
            "sentAt": _utc_now_iso(),
        }

    def on_epoch(self, payload: dict[str, Any]) -> None:
        """Retained ``traffic/epoch``: a newer coordinator voids every grant of an older one."""
        epoch = int(payload.get("epoch") or 0)
        if epoch <= self.known_epoch:
            return
        self.known_epoch = epoch
        if self.grant is not None and self.grant.epoch < epoch:
            self._drop_grant("NEW_EPOCH")
            self._set_ok(False)
        # Re-propose at once so the new coordinator can grant the current plan.
        self._last_propose_mono = -1e9

    def on_link(self, connected: bool) -> None:
        self.link_up = bool(connected)
        if not connected:
            self._set_ok(False)

    # ------------------------------------------------------------------ gate

    def is_grant_valid(self) -> bool:
        return self._grant_block_reason() is None

    def _grant_block_reason(self) -> Optional[str]:
        g = self.grant
        if g is None:
            return self.last_block_reason if self.last_block_reason else "NO_GRANT"
        if not self.link_up:
            return "LINK_DOWN"
        if g.epoch < self.known_epoch:
            return "STALE_EPOCH"
        if self.monotonic() - g.last_heard_mono > self.authority_loss_timeout_sec:
            return "AUTHORITY_SILENT"
        if self.now() >= g.expires_at:
            return "EXPIRED"
        return None

    def allowed_resources(self) -> frozenset[str]:
        """Grant resources the robot may use now: all of them for the granted plan; for a newer plan still
        awaiting its grant, only those the new path also needs (the rest may already be released)."""
        g = self.grant
        if g is None:
            return frozenset()
        if g.plan_revision == self.plan_revision:
            return g.resource_ids
        key = (self.plan_revision, g.authority_id, self.plan_hash)
        if self._allowed_cache is None or self._allowed_cache[0] != key:
            needed = collect_resources(self.plan_poses, self.zones, g.geometry)
            self._allowed_cache = (key, g.resource_ids & needed)
        return self._allowed_cache[1]

    def motion_block_reason(self, pose: Optional[Pose]) -> Optional[str]:
        reason = self._grant_block_reason()
        if reason is not None:
            return reason
        if pose is None:
            return "POSE_UNKNOWN"
        g = self.grant
        assert g is not None
        allowed = self.allowed_resources()
        radius = g.geometry.footprint_radius + max(0.0, self.stop_margin_m)
        if not footprint_resources(pose[0], pose[1], radius, self.zones, g.geometry.cell_size) <= allowed:
            return "OUTSIDE_RESERVATION"
        # Bare footprint, no margin: a released cell was left with exit_margin_m to spare, so localization jitter at
        # the boundary does not latch the gate shut; actually driving back into it does.
        if self.released_resources and (
            footprint_resources(pose[0], pose[1], g.geometry.footprint_radius, self.zones, g.geometry.cell_size)
            & self.released_resources
        ):
            return "IN_RELEASED_RESOURCE"
        return None

    def recovery_block_reason(self, pose: Optional[Pose], kind: str, distance_m: float = 0.0) -> Optional[str]:
        """Why a recovery manoeuvre may not start now (None: it stays inside the grant all the way).

        ``spin``: the footprint radius already covers every heading, so a valid position is enough.
        ``backup``: every pose along the straight reverse line, with ``stop_margin_m``, must be inside the grant and
        out of what this robot already released. The gate keeps checking each tick while the manoeuvre runs.
        """
        reason = self.motion_block_reason(pose)
        if reason is not None:
            return reason
        if kind == "spin":
            return None
        if kind != "backup" or distance_m < 0.0:
            return "UNKNOWN_RECOVERY"
        assert pose is not None and self.grant is not None
        g = self.grant
        allowed = self.allowed_resources() - self.released_resources
        radius = g.geometry.footprint_radius + max(0.0, self.stop_margin_m)
        steps = max(1, math.ceil(distance_m / 0.02))
        for i in range(1, steps + 1):
            d = distance_m * i / steps
            x, y = pose[0] - d * math.cos(pose[2]), pose[1] - d * math.sin(pose[2])
            if not footprint_resources(x, y, radius, self.zones, g.geometry.cell_size) <= allowed:
                return "REVERSE_OUTSIDE_RESERVATION"
        return None

    def tick(self, pose: Optional[Pose]) -> bool:
        """≥20 Hz: publish authority_ok for cmd_vel_authority_gate. Returns the decision."""
        reason = self.motion_block_reason(pose)
        if reason is not None:
            self.last_block_reason = reason
            self._last_blocked_mono = self.monotonic()
        self._set_ok(reason is None)
        return reason is None

    def localization_regained(self, localized: bool) -> bool:
        """True once on not-localized → localized. The gateway reports its state at boot, before the operator has set
        a pose; a backend holding occupancy for this robot quarantines it then, and only a later report with the
        localized pose can lift that."""
        regained = localized and not self._was_localized
        self._was_localized = localized
        return regained

    def state_report_due(self, period_sec: float = 5.0) -> bool:
        """Executing a command without a grant: the backend may hold this robot quarantined until it reports in (a
        single report at reconnect can be lost), so repeat it. True at most once per period."""
        if self.command_id is None or self.grant is not None:
            return False
        if self.monotonic() - self._last_state_report_mono < period_sec:
            return False
        self._last_state_report_mono = self.monotonic()
        return True

    def held_for_traffic(self, window_sec: float = 5.0) -> bool:
        """The gate held the robot during the current command within the last ``window_sec``."""
        return self.command_id is not None and self.monotonic() - self._last_blocked_mono <= window_sec

    # ------------------------------------------------------------------ uplink: verified exits

    def resources_cleared_at(self, x: float, y: float) -> list[str]:
        """Granted resources the footprint entered and has now left (plus margin). Never reports ahead-only ones."""
        g = self.grant
        if g is None:
            return []
        under = footprint_resources(
            x, y, g.geometry.footprint_radius + max(0.0, self.exit_margin_m), self.zones, g.geometry.cell_size
        ) & g.resource_ids
        self.entered_resources |= under
        cleared = sorted(self.entered_resources - under)
        self.entered_resources -= set(cleared)
        self.released_resources |= set(cleared)
        return cleared

    def publish_cleared_resources(self, cleared: list[str], *, x: float, y: float, yaw: float, localization_ok: bool) -> None:
        if not cleared or self.grant is None:
            return
        self._progress_seq += 1
        payload = {
            "schemaVersion": 1,
            "messageId": str(uuid.uuid4()),
            "robotCode": self.robot_code,
            "event": "RESOURCES_CLEARED",
            "authorityId": self.grant.authority_id,
            "epoch": self.grant.epoch,
            "seq": self._progress_seq,
            "progressSeq": self._progress_seq,
            "commandId": self.command_id,
            "poseX": x,
            "poseY": y,
            "poseYaw": yaw,
            "localizationOk": bool(localization_ok),
            "clearedResourceIds": list(cleared),
            "sentAt": _utc_now_iso(),
        }
        self.publish_json(f"{self.topic_prefix.rstrip('/')}/robots/{self.robot_code}/traffic_progress", payload)

    def state_report(self) -> dict[str, Any]:
        """authority_state body: what this robot holds and whether it is executing."""
        g = self.grant
        return {
            "state": "EXECUTING" if self.command_id else "IDLE",
            "activeAuthorityId": g.authority_id if g else None,
            "activeCommandId": self.command_id,
            "lastEpoch": g.epoch if g else self.known_epoch,
            "lastSeq": g.seq if g else 0,
            "grantedEdgeIds": [],
        }

    # ------------------------------------------------------------------ internals

    def _drop_grant(self, reason: str) -> None:
        self.grant = None
        self.entered_resources.clear()
        self.released_resources.clear()
        self._allowed_cache = None
        self.last_block_reason = reason

    def _set_ok(self, ok: bool) -> None:
        self.set_authority_ok(bool(ok))


# ---------------------------------------------------------------------- geometry (mirror of SemanticReservation.cs)


def cell_id(ix: int, iy: int) -> str:
    return f"cell:{ix}:{iy}"


def disc_cells(x: float, y: float, radius: float, cell_size: float, into: set[str]) -> None:
    ix0 = math.floor((x - radius) / cell_size)
    ix1 = math.floor((x + radius) / cell_size)
    iy0 = math.floor((y - radius) / cell_size)
    iy1 = math.floor((y + radius) / cell_size)
    for ix in range(ix0, ix1 + 1):
        for iy in range(iy0, iy1 + 1):
            dx = max(max(ix * cell_size - x, 0.0), x - (ix + 1) * cell_size)
            dy = max(max(iy * cell_size - y, 0.0), y - (iy + 1) * cell_size)
            if dx * dx + dy * dy <= radius * radius:
                into.add(cell_id(ix, iy))


def point_in_ring(x: float, y: float, ring: Sequence[Sequence[float]]) -> bool:
    n = len(ring)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = float(ring[i][0]), float(ring[i][1])
        xj, yj = float(ring[j][0]), float(ring[j][1])
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) if (yj - yi) != 0 else 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx, vy = bx - ax, by - ay
    c2 = vx * vx + vy * vy
    t = 0.0 if c2 <= 0 else min(max((vx * (px - ax) + vy * (py - ay)) / c2, 0.0), 1.0)
    ex = px - (ax + t * vx)
    ey = py - (ay + t * vy)
    return math.sqrt(ex * ex + ey * ey)


def disc_hits_ring(x: float, y: float, radius: float, ring: Sequence[Sequence[float]]) -> bool:
    if point_in_ring(x, y, ring):
        return True
    n = len(ring)
    j = n - 1
    for i in range(n):
        if point_to_segment_distance(x, y, float(ring[j][0]), float(ring[j][1]), float(ring[i][0]), float(ring[i][1])) <= radius:
            return True
        j = i
    return False


def _zone_rings(zones: Iterable[dict[str, Any]]) -> Iterable[tuple[str, list]]:
    for z in zones:
        code = str(z.get("code") or "")
        if not code:
            continue
        for ring in z.get("rings") or []:
            if len(ring) >= 3:
                yield code, ring


def sample_path(poses: Sequence[Sequence[float]], step: float, horizon: float) -> list[tuple[float, float]]:
    if not poses:
        return []
    samples = [(float(poses[0][0]), float(poses[0][1]))]
    traveled = 0.0
    i = 1
    while i < len(poses) and traveled <= horizon:
        ax, ay = float(poses[i - 1][0]), float(poses[i - 1][1])
        bx, by = float(poses[i][0]), float(poses[i][1])
        length = math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay))
        i += 1
        if length <= 0:
            continue
        n = math.ceil(length / step)
        for k in range(1, n + 1):
            t = k / n
            samples.append((ax + (bx - ax) * t, ay + (by - ay) * t))
        traveled += length
    return samples


def collect_resources(poses: Sequence[Sequence[float]], zones: list[dict[str, Any]], geometry: SemanticGeometry) -> frozenset[str]:
    """Same zones/cells the backend reserves for this path (first ``horizon`` metres)."""
    out: set[str] = set()
    rings = list(_zone_rings(zones))
    for x, y in sample_path(poses, geometry.sample_step, max(geometry.cell_size, geometry.horizon)):
        for code, ring in rings:
            if disc_hits_ring(x, y, geometry.reserve_radius, ring):
                out.add(f"zone:{code}")
        disc_cells(x, y, geometry.reserve_radius, geometry.cell_size, out)
    return frozenset(out)


def footprint_resources(x: float, y: float, radius: float, zones: list[dict[str, Any]], cell_size: float) -> frozenset[str]:
    """Cells and zones the robot (any heading) touches at (x, y)."""
    out: set[str] = set()
    disc_cells(x, y, radius, cell_size, out)
    for code, ring in _zone_rings(zones):
        if disc_hits_ring(x, y, radius, ring):
            out.add(f"zone:{code}")
    return frozenset(out)


def plan_hash(poses: Sequence[Sequence[float]]) -> str:
    """Quantized to ~10 cm / ~6 deg so Nav2 micro-replans do not churn planRevision."""
    pts = [[round(float(p[0]), 1), round(float(p[1]), 1), round(float(p[2]), 1)] for p in poses]
    raw = json.dumps(pts, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def _parse_expires_at(expires: Any) -> float:
    """Backend ``expiresAt`` (ISO-8601). Missing / unparseable → already expired (fail closed)."""
    if isinstance(expires, str) and expires:
        try:
            text = expires.strip()
            if text.endswith("Z"):
                text = text[:-1] + "+00:00"
            # .NET writes 7 fractional digits; fromisoformat (3.10) accepts at most 6.
            if "." in text:
                head, _, rest = text.partition(".")
                digits = "".join(ch for ch in rest if ch.isdigit())
                tz = rest[len(digits):]
                text = f"{head}.{digits[:6]}{tz}"
            dt = datetime.fromisoformat(text)
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=timezone.utc)
            return dt.timestamp()
        except ValueError:
            pass
    return 0.0
