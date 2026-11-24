"""AUTO_LOCALIZE command: the robot finds where it stands by itself, and only takes a pose it is sure of.

Order, safest first:
  1. Scan matching against the candidates the backend sent (last pose, home endpoint, every endpoint), standing still
     (scan_match_localizer.py). A confident match is published on /initialpose, then the localizer (AMCL or
     slam_toolbox) must converge on it.
  2. Only when the operator allowed it (payload.allowSpin): global localization, the robot turning on the spot while
     the localizer spreads its estimate over the whole map, then the converged pose is checked against the scan again.
     Not every localizer can do this (slam_toolbox cannot): then step 2 is skipped.
  3. Otherwise nothing is taken: the result is FAILED with the best guesses (``suggestions``) so a person can confirm
     one with a single click (LOCALIZE at that pose).

Ack first, then the work runs off the MQTT thread; the result says how it ended (``method`` CANDIDATE_MATCH /
GLOBAL_SPIN / NONE, the pose, its score and margin, the suggestions).
"""

from __future__ import annotations

import math
import threading
from typing import Any, Callable, Optional, Sequence

from warotrans_fleet.command_contract import (
    OUTCOME_FAILED,
    OUTCOME_SUCCEEDED,
    REASON_BUSY,
    REASON_INVALID_PAYLOAD,
    CommandEnvelope,
    CommandRejected,
)
from warotrans_fleet.command_router import AckDecision, CommandSession, LogSink
from warotrans_fleet.scan_match_localizer import Candidate, LikelihoodField, MatchResult, MatchSettings, match

AUTO_LOCALIZE = "AUTO_LOCALIZE"

METHOD_CANDIDATE = "CANDIDATE_MATCH"
METHOD_SPIN = "GLOBAL_SPIN"
METHOD_NONE = "NONE"

ERROR_NO_SCAN = "LOCALIZATION_NO_SCAN"
ERROR_NO_MAP = "LOCALIZATION_NO_MAP"
ERROR_NO_MATCH = "LOCALIZATION_NO_MATCH"
ERROR_AMBIGUOUS = "LOCALIZATION_AMBIGUOUS"
ERROR_NOT_CONVERGED = "LOCALIZATION_NOT_CONVERGED"
ERROR_SPIN_FAILED = "LOCALIZATION_SPIN_FAILED"

Points = Sequence[tuple[float, float]]


def parse_candidates(payload: dict[str, Any]) -> list[Candidate]:
    raw = payload.get("candidates")
    if not isinstance(raw, list) or not raw:
        raise CommandRejected(REASON_INVALID_PAYLOAD, "candidates must be a non-empty list")
    out: list[Candidate] = []
    for item in raw[:200]:
        if not isinstance(item, dict):
            continue
        try:
            x, y = float(item["x"]), float(item["y"])
        except (KeyError, TypeError, ValueError):
            continue
        yaw = item.get("yaw")
        out.append(
            Candidate(
                x,
                y,
                float(yaw) if isinstance(yaw, (int, float)) else None,
                item.get("endpointCode") if isinstance(item.get("endpointCode"), str) else None,
                str(item.get("source") or "ENDPOINT"),
            )
        )
    if not out or not all(math.isfinite(c.x) and math.isfinite(c.y) for c in out):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "no valid candidate pose")
    return out


def _settings(payload: dict[str, Any]) -> MatchSettings:
    s = MatchSettings()
    for key, attr in (("minScore", "min_score"), ("minMargin", "min_margin")):
        value = payload.get(key)
        if isinstance(value, (int, float)) and 0.0 < float(value) < 1.0:
            setattr(s, attr, float(value))
    return s


class AutoLocalizeCommandHandler:
    command_type = AUTO_LOCALIZE

    def __init__(
        self,
        *,
        busy: Callable[[], bool],
        get_points: Callable[[float], Optional[Points]],
        get_field: Callable[[], Optional[LikelihoodField]],
        publish_initial_pose: Callable[[float, float, float], None],
        wait_converged: Callable[[float], Optional[str]],
        log: LogSink,
        global_localize: Optional[Callable[[], Optional[str]]] = None,
        current_pose: Optional[Callable[[], Optional[tuple[float, float, float]]]] = None,
        run_async: Optional[Callable[[Callable[[], None]], None]] = None,
        converge_timeout_sec: float = 10.0,
        scan_timeout_sec: float = 3.0,
    ) -> None:
        self._busy = busy
        self._get_points = get_points
        self._get_field = get_field
        self._publish = publish_initial_pose
        self._wait_converged = wait_converged
        self._global_localize = global_localize
        self._current_pose = current_pose
        self._run_async = run_async or (lambda fn: threading.Thread(target=fn, name="auto-localize", daemon=True).start())
        self._log = log
        self._converge_timeout = converge_timeout_sec
        self._scan_timeout = scan_timeout_sec
        self._lock = threading.Lock()
        self._running = False

    @property
    def running(self) -> bool:
        return self._running

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        if self._busy():
            raise CommandRejected(REASON_BUSY, "a MOVE is running; stop or cancel it before localizing")
        payload = command.payload if isinstance(command.payload, dict) else {}
        candidates = parse_candidates(payload)
        settings = _settings(payload)
        allow_spin = bool(payload.get("allowSpin"))
        with self._lock:
            if self._running:
                raise CommandRejected(REASON_BUSY, "an auto-localize is already running")
            self._running = True

        def _work() -> None:
            try:
                outcome, error, result = self._localize(candidates, settings, allow_spin)
            except Exception as exc:  # never leave the command open
                self._log.error(f"[AutoLocalize] failed: {exc!r}")
                outcome, error, result = OUTCOME_FAILED, "INTERNAL_ERROR", {"method": METHOD_NONE}
            finally:
                with self._lock:
                    self._running = False
            self._log.info(f"[AutoLocalize] {outcome} error={error} result={result}")
            session.complete(outcome, error_code=error, result=result)

        return AckDecision.accept(after_ack=lambda: self._run_async(_work))

    # ------------------------------------------------------------------ the work

    def _localize(
        self, candidates: list[Candidate], settings: MatchSettings, allow_spin: bool
    ) -> tuple[str, Optional[str], dict[str, Any]]:
        field = self._get_field()
        if field is None:
            return OUTCOME_FAILED, ERROR_NO_MAP, {"method": METHOD_NONE}
        points = self._get_points(self._scan_timeout)
        if not points:
            return OUTCOME_FAILED, ERROR_NO_SCAN, {"method": METHOD_NONE}

        found = match(field, points, candidates, settings)
        summary = _summary(found)
        if found.confident and found.best is not None:
            best = found.best
            self._publish(best.x, best.y, best.yaw)
            reason = self._wait_converged(self._converge_timeout)
            if reason is None:
                return OUTCOME_SUCCEEDED, None, {**summary, "method": METHOD_CANDIDATE, **_pose(best.x, best.y, best.yaw)}
            return OUTCOME_FAILED, ERROR_NOT_CONVERGED, {**summary, "method": METHOD_NONE, "detail": reason}

        if allow_spin and self._global_localize is not None and self._current_pose is not None:
            reason = self._global_localize()
            if reason is not None:
                return OUTCOME_FAILED, ERROR_SPIN_FAILED, {**summary, "method": METHOD_NONE, "detail": reason}
            pose = self._current_pose()
            fresh = self._get_points(self._scan_timeout)
            if pose is not None and fresh:
                # The localizer converging is not enough (a look-alike corner converges too): the pose it settled on
                # must pass the same test as a candidate, the whole-map check included.
                check = match(field, fresh, [Candidate(pose[0], pose[1], pose[2], None, "SPIN")], settings)
                near = check.best is not None and math.hypot(check.best.x - pose[0], check.best.y - pose[1]) < settings.same_place_m
                if check.confident and near:
                    return OUTCOME_SUCCEEDED, None, {
                        **_summary(check), "method": METHOD_SPIN, **_pose(*pose),
                    }
                detail = f"spin pose not confirmed by the scan ({check.reason})"
                return OUTCOME_FAILED, ERROR_NO_MATCH, {**_summary(check), "method": METHOD_NONE, "detail": detail}
            return OUTCOME_FAILED, ERROR_SPIN_FAILED, {**summary, "method": METHOD_NONE, "detail": "no pose after spin"}

        error = ERROR_AMBIGUOUS if found.reason == "AMBIGUOUS" else ERROR_NO_MATCH
        if found.reason == "NO_SCAN":
            error = ERROR_NO_SCAN
        return OUTCOME_FAILED, error, {**summary, "method": METHOD_NONE}


def _pose(x: float, y: float, yaw: float) -> dict[str, float]:
    return {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)}


def _summary(found: MatchResult) -> dict[str, Any]:
    best = found.best
    return {
        "reason": found.reason,
        "score": round(best.score, 3) if best else None,
        "margin": round(found.margin, 3) if found.margin is not None else None,
        "endpointCode": best.code if best else None,
        "suggestions": [h.to_dict() for h in found.suggestions],
    }
