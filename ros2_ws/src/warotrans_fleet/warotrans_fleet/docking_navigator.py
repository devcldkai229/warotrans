"""NavigatorPort that docks: one NAVIGATE goal becomes up to three Nav2 actions when a bay is too small to turn in.

  1. exit    FollowPath, straight out of the bay the robot stands in (reverse controller when backing out)
  2. travel  NavigateToPose to the turning point at the endpoint heading (or to the endpoint itself)
  3. run in  FollowPath, straight onto the endpoint (reverse controller when backing in)

  4. align   NavigateToPose onto the endpoint itself, a last small in-place turn if the heading is off

Getting unstuck (re-dock, at most max_redocks times per goal): when the run in or the align fails, when an exit
fails, or when Nav2 gives up within redock_radius_m of the endpoint (typically: it could not turn there although the
static map says it can), the plan is made again from where the robot stands: straight out to a point with a wider
margin, turn there, straight in again, align.

MoveCommandHandler still sees a single goal: one response, feedback, one result. A cancel stops the stage that is
running and no later stage starts. Before a FollowPath stage the straight path is proposed to traffic, as Nav2's own
plan is for the travel stage. Geometry decisions live in docking.py.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol, Sequence

from warotrans_fleet.docking import DockingGeometry, DockPlan, Pose, StraightMove
from warotrans_fleet.navigator import (
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    GoalCallbacks,
    GoalTicket,
    NavFeedback,
    NavGoal,
    NavigatorPort,
    NavResult,
)

CONTROLLER_FORWARD = "FollowPath"
CONTROLLER_REVERSE = "FollowPathReverse"


class FollowPathPort(Protocol):
    def is_ready(self) -> bool:
        ...

    def send_path(
        self, command_id: str, frame_id: str, poses: Sequence[Pose], controller_id: str, ticket: GoalTicket,
        callbacks: GoalCallbacks,
    ) -> None:
        ...

    def cancel_goal(self, ticket: GoalTicket, on_response: Callable[[bool, str], None]) -> None:
        ...


@dataclass
class _Stage:
    kind: str  # "exit" | "travel" | "run_in" | "align"
    goal: Optional[NavGoal] = None
    move: Optional[StraightMove] = None


@dataclass
class _Run:
    goal: NavGoal
    callbacks: GoalCallbacks
    stages: list[_Stage]
    index: int = 0
    inner: GoalTicket = field(default_factory=GoalTicket)
    responded: bool = False
    cancelled: bool = False
    finished: bool = False
    redocks: int = 0
    epoch: int = 0  # bumped by a re-dock: callbacks of the stages before it are stale


class DockingNavigator:
    def __init__(
        self,
        *,
        inner: NavigatorPort,
        follow: FollowPathPort,
        geometry: Optional[DockingGeometry],
        pose_provider: Callable[[], Optional[Pose]],
        propose_path: Callable[[Sequence[Pose]], None] = lambda poses: None,
        log: Callable[[str], None] = lambda text: None,
        arrive_xy_m: float = 0.04,
        align_yaw_rad: float = 0.06,
        max_redocks: int = 2,
        redock_radius_m: float = 0.6,
    ) -> None:
        self._arrive_xy = float(arrive_xy_m)
        self._align_yaw = float(align_yaw_rad)
        self._max_redocks = max(0, int(max_redocks))
        self._redock_radius = float(redock_radius_m)
        self._inner = inner
        self._follow = follow
        self._geometry = geometry
        self._pose = pose_provider
        self._propose = propose_path
        self._log = log
        self._lock = threading.RLock()

    def is_ready(self) -> bool:
        return self._inner.is_ready()

    # ------------------------------------------------------------------ planning
    def _plan(self, goal: NavGoal, **options) -> Optional[DockPlan]:
        if self._geometry is None:
            return None
        start = self._current_pose()
        if start is None:
            return None
        try:
            plan = self._geometry.plan(start, (goal.x, goal.y, goal.yaw), **options)
        except Exception as exc:  # noqa: BLE001
            self._log(f"[Docking] planning failed ({type(exc).__name__}: {exc}); plain goal")
            return None
        return None if plan.plain else plan

    def send_goal(self, goal: NavGoal, ticket: GoalTicket, callbacks: GoalCallbacks) -> None:
        if self._geometry is None:
            self._inner.send_goal(goal, ticket, callbacks)
            return
        plan = self._plan(goal)
        # A plain goal runs as a one-stage run too, so a failure next to the endpoint can still re-dock.
        stages = self._stages(goal, plan) if plan is not None else [_Stage("travel", goal=goal)]
        run = _Run(goal=goal, callbacks=callbacks, stages=stages)
        ticket.handle = run
        if plan is not None:
            self._log(
                f"[Docking] commandId={goal.command_id} goal=({goal.x:.3f},{goal.y:.3f},{goal.yaw:.3f}) stages="
                + ", ".join(self._describe(s) for s in stages)
            )
        self._start_stage(run)

    @staticmethod
    def _stages(goal: NavGoal, plan: DockPlan) -> list[_Stage]:
        stages: list[_Stage] = []
        if plan.exit is not None:
            stages.append(_Stage("exit", move=plan.exit))
        if plan.approach is not None:
            ax, ay, ayaw = plan.approach
            stages.append(_Stage("travel", goal=NavGoal(goal.command_id, goal.frame_id, ax, ay, ayaw)))
        elif plan.run_in is None:
            stages.append(_Stage("travel", goal=goal))
        if plan.run_in is not None:
            stages.append(_Stage("run_in", move=plan.run_in))
            # A straight move cannot correct the heading: a last small in-place turn on the endpoint, if needed.
            stages.append(_Stage("align", goal=goal))
        return stages

    @staticmethod
    def _describe(stage: _Stage) -> str:
        if stage.move is not None:
            m = stage.move
            return f"{stage.kind} {m.length:.2f}m {'reverse' if m.reverse else 'forward'}"
        g = stage.goal
        return f"{stage.kind} ({g.x:.3f},{g.y:.3f},{g.yaw:.3f})"

    # ------------------------------------------------------------------ stages
    def _stage_callbacks(self, run: _Run, index: int) -> GoalCallbacks:
        epoch = run.epoch
        return GoalCallbacks(
            on_response=lambda accepted, error: self._on_response(run, index, accepted, error, epoch),
            on_feedback=lambda fb: self._on_feedback(run, index, fb, epoch),
            on_result=lambda result: self._on_result(run, index, result, epoch),
        )

    def _start_stage(self, run: _Run) -> None:
        with self._lock:
            if run.cancelled or run.finished:
                return
            stage = run.stages[run.index]
            run.inner = GoalTicket()
            callbacks = self._stage_callbacks(run, run.index)
        if stage.kind == "align":
            here = self._current_pose()
            g = stage.goal
            if here is not None and math.hypot(g.x - here[0], g.y - here[1]) <= self._arrive_xy and abs(
                    math.atan2(math.sin(here[2] - g.yaw), math.cos(here[2] - g.yaw))) <= self._align_yaw:
                self._log(f"[Docking] commandId={run.goal.command_id} align skipped: on the endpoint")
                self._skip(run)
                return
        if stage.goal is not None:
            self._inner.send_goal(stage.goal, run.inner, callbacks)
            return
        move = stage.move
        if stage.kind == "run_in":
            # From where the robot really stopped, not the planned turning point; nothing to drive if it is already
            # on the endpoint (a run in shorter than the goal tolerance only crawls against the collision monitor).
            here = self._current_pose()
            if here is not None:
                remaining = math.hypot(move.end[0] - here[0], move.end[1] - here[1])
                if remaining <= self._arrive_xy:
                    self._log(f"[Docking] commandId={run.goal.command_id} run_in skipped: {remaining:.3f} m to go")
                    self._skip(run)
                    return
                along = (move.end[0] - here[0]) * math.cos(move.yaw) + (move.end[1] - here[1]) * math.sin(move.yaw)
                move = StraightMove((here[0], here[1]), move.end, move.yaw, reverse=along < 0)
        poses = move.poses()
        try:
            self._propose(poses)
        except Exception as exc:  # noqa: BLE001
            self._log(f"[Docking] traffic proposal failed: {type(exc).__name__}: {exc}")
        controller = CONTROLLER_REVERSE if move.reverse else CONTROLLER_FORWARD
        self._follow.send_path(run.goal.command_id, run.goal.frame_id, poses, controller, run.inner, callbacks)

    def _close_enough(self, run: _Run, index: int) -> bool:
        """A run in that stopped short (the controller stalls a few cm from a wall) is good enough for the align step."""
        stage = run.stages[index]
        if stage.kind != "run_in" or run.cancelled:
            return False
        here = self._current_pose()
        return here is not None and math.hypot(stage.move.end[0] - here[0], stage.move.end[1] - here[1]) <= self._arrive_xy

    def _current_pose(self) -> Optional[Pose]:
        try:
            return self._pose()
        except Exception:  # noqa: BLE001
            return None

    def _skip(self, run: _Run) -> None:
        """The running stage has nothing to do: on to the next one, or the goal is done."""
        with self._lock:
            first = not run.responded
            run.responded = True
            last = run.index + 1 >= len(run.stages)
            if not last:
                run.index += 1
        if first:
            run.callbacks.on_response(True, None)
        if last:
            self._finish(run, NavResult(status=STATUS_SUCCEEDED))
        else:
            self._start_stage(run)

    def _redock(self, run: _Run, index: int, result: NavResult) -> bool:
        """Make the plan again from here after stage index failed; False when that is not possible or allowed."""
        stage = run.stages[index]
        if run.cancelled or result.status == STATUS_CANCELED or run.redocks >= self._max_redocks:
            return False
        here = self._current_pose()
        if here is None:
            return False
        g = run.goal
        if stage.kind == "travel" and math.hypot(g.x - here[0], g.y - here[1]) > self._redock_radius:
            return False  # failed on the way, not at the endpoint: Nav2's own recovery already had its turn
        avoid = stage.move.reverse if stage.kind == "exit" else None
        plan = self._plan(g, force_dock=True, refine=False, avoid_exit_reverse=avoid)
        if plan is None or plan.run_in is None:
            self._log(f"[Docking] commandId={g.command_id} {stage.kind} failed; no way to dock again from "
                      f"({here[0]:.3f},{here[1]:.3f},{here[2]:.3f})")
            return False
        stages = self._stages(g, plan)
        with self._lock:
            if run.cancelled or run.finished:
                return False
            run.redocks += 1
            run.epoch += 1
            run.stages = stages
            run.index = 0
        self._log(
            f"[Docking] commandId={g.command_id} redock {run.redocks}/{self._max_redocks} after {stage.kind} "
            f"status={result.status} from ({here[0]:.3f},{here[1]:.3f},{here[2]:.3f}); stages="
            + ", ".join(self._describe(s) for s in stages)
        )
        self._start_stage(run)
        return True

    def _on_response(self, run: _Run, index: int, accepted: bool, error: Optional[str], epoch: int = 0) -> None:
        with self._lock:
            if index != run.index or epoch != run.epoch or run.finished:
                return
            first = not run.responded
            run.responded = True
        if not accepted:
            self._finish(run, None, accepted=False, error=error)
            return
        if first:
            run.callbacks.on_response(True, None)

    def _on_feedback(self, run: _Run, index: int, feedback: NavFeedback, epoch: int = 0) -> None:
        with self._lock:
            if index != run.index or epoch != run.epoch or run.finished:
                return
            # Distance still to cover: this stage plus the straight stages after it.
            rest = sum(s.move.length for s in run.stages[index + 1:] if s.move is not None)
        d = feedback.distance_remaining
        run.callbacks.on_feedback(NavFeedback(
            distance_remaining=(d + rest) if d is not None else None,
            navigation_time_sec=feedback.navigation_time_sec,
            estimated_time_remaining_sec=feedback.estimated_time_remaining_sec,
            number_of_recoveries=feedback.number_of_recoveries,
        ))

    def _on_result(self, run: _Run, index: int, result: NavResult, epoch: int = 0) -> None:
        with self._lock:
            if index != run.index or epoch != run.epoch or run.finished:
                return
            last = index + 1 >= len(run.stages)
            ok = result.status == STATUS_SUCCEEDED or self._close_enough(run, index)
            if ok and not last and not run.cancelled:
                run.index += 1
                self._log(f"[Docking] commandId={run.goal.command_id} {run.stages[index].kind} done; "
                          f"{self._describe(run.stages[run.index])}")
                go_on = True
            else:
                go_on = False
        if go_on:
            self._start_stage(run)
            return
        if result.status != STATUS_SUCCEEDED:
            if self._redock(run, index, result):
                return
            self._log(f"[Docking] commandId={run.goal.command_id} {run.stages[index].kind} ended status={result.status}")
        self._finish(run, result)

    def _finish(self, run: _Run, result: Optional[NavResult], *, accepted: bool = True, error: Optional[str] = None) -> None:
        with self._lock:
            if run.finished:
                return
            run.finished = True
            responded = run.responded
        if result is None:
            if not responded:
                run.callbacks.on_response(False, error)
            else:
                run.callbacks.on_result(NavResult(status=STATUS_CANCELED if run.cancelled else 6,
                                                  error_message=error or "stage rejected"))
            return
        run.callbacks.on_result(result)

    # ------------------------------------------------------------------ cancel
    def cancel_goal(self, ticket: GoalTicket, on_response: Callable[[bool, str], None]) -> None:
        run = ticket.handle
        if not isinstance(run, _Run):
            self._inner.cancel_goal(ticket, on_response)
            return
        with self._lock:
            run.cancelled = True
            stage = run.stages[run.index]
            inner = run.inner
        port = self._inner if stage.goal is not None else self._follow
        port.cancel_goal(inner, on_response)

