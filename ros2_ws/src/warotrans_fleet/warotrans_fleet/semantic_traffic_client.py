"""Stage-3 semantic_active traffic client: propose Nav2 plan, gate on Backend grant."""

from __future__ import annotations

import hashlib
import json
import math
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Optional, Sequence


@dataclass
class SemanticGrant:
    authority_id: str
    epoch: int
    seq: int
    fingerprint: str
    plan_revision: int
    resource_ids: list[str]
    path_hash: str
    expires_at: Optional[float] = None  # epoch seconds


@dataclass
class SemanticTrafficClient:
    robot_code: str
    topic_prefix: str
    fingerprint: str
    publish_json: Callable[[str, dict[str, Any]], None]
    set_authority_ok: Callable[[bool], None]
    now: Callable[[], float] = time.time
    authority_loss_timeout_sec: float = 3.0

    plan_revision: int = 0
    grant: Optional[SemanticGrant] = None
    last_proposed_poses: list[tuple[float, float, float]] = field(default_factory=list)
    last_path_fingerprint: str = ""
    held_resources: set[str] = field(default_factory=set)
    # Resources the footprint has actually entered — only these may be reported cleared.
    entered_resources: set[str] = field(default_factory=set)
    _progress_seq: int = 0
    _last_propose_monotonic: float = 0.0
    propose_retry_sec: float = 2.0
    # Host replan: revoke MQTT then EXTEND — gap closed the gate and aborted progress (~0.85 m).
    replan_grace_sec: float = 3.0
    _replan_grace_until_mono: float = 0.0
    _authority_latched: bool = False

    def on_plan(self, poses: Sequence[tuple[float, float, float]], *, goal: Optional[tuple[float, float, float]] = None,
                command_id: Optional[str] = None, job_step_id: Optional[str] = None, job_id: Optional[str] = None) -> None:
        """Called when Nav2 publishes a new /plan. Bumps revision only when path changes."""
        if len(poses) < 2:
            return
        path_fp = _path_fingerprint(poses)
        # Debounce identical replans: keep gate open under the current grant.
        if path_fp == self.last_path_fingerprint and self.grant is not None and self.is_grant_valid():
            return
        # Same path without a grant: re-propose on a cooldown WITHOUT bumping planRevision
        # (bumping made in-flight grants always stale → authority_ok stuck False).
        same_path_retry = path_fp == self.last_path_fingerprint and self.plan_revision > 0
        if same_path_retry:
            if (time.monotonic() - self._last_propose_monotonic) < self.propose_retry_sec:
                return
        else:
            grant_was_current = self.grant is not None and self.grant.plan_revision == self.plan_revision
            self.plan_revision += 1
            if grant_was_current:
                # Handoff starts once, when the held grant stops matching the path. Later replans while it is
                # outstanding must NOT extend it: a moving robot replans continuously, so extending would let a
                # refused path keep driving on the old reservation indefinitely.
                self._replan_grace_until_mono = time.monotonic() + self.replan_grace_sec

        self.last_proposed_poses = list(poses)
        self.last_path_fingerprint = path_fp
        self._last_propose_monotonic = time.monotonic()
        # Soft handoff: keep a valid grant (and grace latch) across Nav2/Host replan gaps.
        if not (self.grant is not None and self.is_grant_valid()):
            self.grant = None
            if not self._authority_latched:
                self._set_ok(False)
        payload = {
            "schemaVersion": 1,
            "messageId": str(uuid.uuid4()),
            "robotCode": self.robot_code,
            "semanticMapFingerprint": self.fingerprint,
            "planRevision": self.plan_revision,
            "commandId": command_id,
            "jobStepId": job_step_id,
            "jobId": job_id,
            "poses": [{"x": p[0], "y": p[1], "yaw": p[2]} for p in poses],
            "goal": None if goal is None else {"x": goal[0], "y": goal[1], "yaw": goal[2]},
            "sentAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        topic = f"{self.topic_prefix.rstrip('/')}/robots/{self.robot_code}/traffic_plan"
        self.publish_json(topic, payload)

    def on_authority_payload(self, authority: dict[str, Any]) -> bool:
        """Apply EXTEND/NAVIGATE authority payload. Returns True if gate may open."""
        fp = str(authority.get("semanticMapFingerprint") or authority.get("graphHash") or "")
        rev = int(authority.get("planRevision") or 0)
        if fp and self.fingerprint and fp != self.fingerprint:
            self._set_ok(False)
            return False
        if rev <= 0:
            # A semantic grant authorizes one proposed plan; without a revision it cannot be matched to it.
            return False
        if rev != self.plan_revision:
            # Stale grant for an older planRevision. Ignore — do not drop a valid in-flight
            # grant (soft handoff while EXTEND for the new revision is outstanding).
            return False
        epoch = int(authority.get("epoch") or 0)
        seq = int(authority.get("seq") or 0)
        authority_id = str(authority.get("authorityId") or "")
        if self.grant is not None and (
            epoch < self.grant.epoch
            or (authority_id == self.grant.authority_id and epoch == self.grant.epoch and seq <= self.grant.seq)
        ):
            # QoS1 redelivery or an older epoch: never let it reopen the gate or roll the grant back.
            return False
        exp_ts = _parse_expires_at(authority.get("expiresAt"), self.now, self.authority_loss_timeout_sec)
        if exp_ts <= self.now():
            # Expired on arrival (late delivery or robot clock ahead): checked BEFORE the gate opens.
            return False
        resources = [str(x) for x in (authority.get("resourceIds") or [])]
        self.grant = SemanticGrant(
            authority_id=authority_id,
            epoch=epoch,
            seq=seq,
            fingerprint=fp or self.fingerprint,
            plan_revision=rev,
            resource_ids=resources,
            path_hash=str(authority.get("pathHash") or ""),
            expires_at=exp_ts,
        )
        self.held_resources = set(resources)
        # Keep entered ∩ new grant; do not treat ahead-only resources as clearable yet.
        self.entered_resources &= self.held_resources
        self._progress_seq = max(self._progress_seq, self.grant.seq)
        self._authority_latched = True
        self._replan_grace_until_mono = 0.0
        self._set_ok(True)
        return True

    def on_renew(self, payload: dict[str, Any]) -> None:
        if self.grant is None:
            return
        if str(payload.get("authorityId") or "") != self.grant.authority_id:
            return
        if int(payload.get("epoch") or 0) != self.grant.epoch:
            return
        exp_ts = _parse_expires_at(payload.get("expiresAt"), self.now, self.authority_loss_timeout_sec)
        if exp_ts <= self.now():
            return
        self.grant.expires_at = exp_ts
        self._authority_latched = True
        self._set_ok(True)

    def on_revoke(self) -> None:
        """REVOKE_AUTHORITY stops the robot now.

        The backend only sends REVOKE from a quarantine (mode STOP_NOW); a replan re-acquire revokes
        internally without a message, so there is no revoke→EXTEND gap to bridge here.
        """
        self.grant = None
        self.held_resources.clear()
        self.entered_resources.clear()
        self._authority_latched = False
        self._replan_grace_until_mono = 0.0
        self._set_ok(False)

    def is_grant_valid(self) -> bool:
        if self.grant is None:
            return False
        if self.grant.expires_at is not None and self.now() > self.grant.expires_at:
            return False
        if self.grant.plan_revision != self.plan_revision and time.monotonic() >= self._replan_grace_until_mono:
            # The grant reserved the PREVIOUS path. It bridges a replan only for replan_grace_sec; after that a
            # path the backend did not grant (e.g. refused for a conflict) must not keep driving on it.
            return False
        return True

    def tick(self) -> None:
        """Watchdog + liveness: republish authority_ok while grant is valid (≥20 Hz from caller)."""
        if self.is_grant_valid():
            self._authority_latched = True
            self._set_ok(True)
            return
        if self.grant is not None:
            self.grant = None
        if self._authority_latched and time.monotonic() < self._replan_grace_until_mono:
            # Keep gate open across Host revoke→grant gap.
            self.set_authority_ok(True)
            return
        self._authority_latched = False
        self._set_ok(False)

    def publish_cleared_resources(
        self,
        cleared: list[str],
        *,
        x: float,
        y: float,
        yaw: float,
        localization_ok: bool,
    ) -> None:
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
            "poseX": x,
            "poseY": y,
            "poseYaw": yaw,
            "localizationOk": bool(localization_ok),
            "clearedResourceIds": list(cleared),
            "sentAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        topic = f"{self.topic_prefix.rstrip('/')}/robots/{self.robot_code}/traffic_progress"
        self.publish_json(topic, payload)

    def resources_cleared_at(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        zones: list[dict[str, Any]],
        cell_size: float = 0.5,
        half_l: float = 0.18,
        half_w: float = 0.15,
    ) -> list[str]:
        """Return held resources that were entered and whose footprint no longer intersects pose.

        Ahead-only reservations (never intersected) are NOT reported cleared.
        """
        under = _resources_under_footprint(
            x, y, yaw, self.held_resources, zones, cell_size, half_l, half_w
        )
        self.entered_resources |= under
        cleared: list[str] = []
        still: set[str] = set()
        for rid in list(self.held_resources):
            if rid in under:
                still.add(rid)
                continue
            if rid in self.entered_resources:
                cleared.append(rid)
                self.entered_resources.discard(rid)
            else:
                # Reserved ahead — keep held until entered then exited (or backend diff-release).
                still.add(rid)
        self.held_resources = still
        return cleared

    def _set_ok(self, ok: bool) -> None:
        self.set_authority_ok(bool(ok))


def _resources_under_footprint(
    x: float,
    y: float,
    yaw: float,
    held: set[str],
    zones: list[dict[str, Any]],
    cell_size: float,
    half_l: float,
    half_w: float,
) -> set[str]:
    under: set[str] = set()
    cells = _footprint_cells(x, y, yaw, half_l, half_w, cell_size)
    for rid in held:
        if rid.startswith("cell:") and rid in cells:
            under.add(rid)
        elif rid.startswith("zone:"):
            code = rid[5:]
            for z in zones:
                if z.get("code") != code:
                    continue
                for ring in z.get("rings") or []:
                    if _footprint_hits(x, y, yaw, ring, half_l, half_w):
                        under.add(rid)
                        break
    return under


def _path_fingerprint(poses: Sequence[tuple[float, float, float]]) -> str:
    """Quantize path to ~10 cm / ~6 deg so Nav2 micro-replans do not thrash planRevision/gate."""
    pts = []
    for x, y, yaw in poses:
        pts.append([round(x, 1), round(y, 1), round(yaw, 1)])
    raw = json.dumps(pts, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()


def _parse_expires_at(expires: Any, now: Callable[[], float], fallback_sec: float) -> float:
    if isinstance(expires, str) and expires:
        try:
            text = expires.strip()
            if text.endswith("Z"):
                text = text[:-1] + "+00:00"
            dt = datetime.fromisoformat(text)
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=timezone.utc)
            return dt.timestamp()
        except ValueError:
            pass
    return now() + max(0.5, float(fallback_sec))


def _point_in_ring(x: float, y: float, ring: list) -> bool:
    n = len(ring)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = float(ring[i][0]), float(ring[i][1])
        xj, yj = float(ring[j][0]), float(ring[j][1])
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def _footprint_hits(x: float, y: float, yaw: float, ring: list, half_l: float, half_w: float) -> bool:
    c, s = math.cos(yaw), math.sin(yaw)
    for lx, ly in ((half_l, half_w), (half_l, -half_w), (-half_l, half_w), (-half_l, -half_w), (0.0, 0.0)):
        wx = x + c * lx - s * ly
        wy = y + s * lx + c * ly
        if _point_in_ring(wx, wy, ring):
            return True
    return False


def _footprint_cells(x: float, y: float, yaw: float, half_l: float, half_w: float, cell_size: float) -> set[str]:
    c, s = math.cos(yaw), math.sin(yaw)
    out: set[str] = set()
    step = min(half_l, cell_size * 0.5) or cell_size
    step_w = min(half_w, cell_size * 0.5) or cell_size
    u = -half_l
    while u <= half_l + 1e-9:
        v = -half_w
        while v <= half_w + 1e-9:
            wx = x + c * u - s * v
            wy = y + s * u + c * v
            ix = int(math.floor(wx / cell_size))
            iy = int(math.floor(wy / cell_size))
            out.add(f"cell:{ix}:{iy}")
            v += step_w
        u += step
    return out
