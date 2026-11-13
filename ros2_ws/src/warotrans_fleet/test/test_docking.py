"""Docking (2026-11-11 lab): turn where there is room on the bay axis, drive straight in and out."""

import math
import os

import pytest
from fleet_fakes import FakeNavigator

from warotrans_fleet.docking import DockingGeometry, OccupancyMap, StraightMove
from warotrans_fleet.docking_navigator import CONTROLLER_FORWARD, CONTROLLER_REVERSE, DockingNavigator
from warotrans_fleet.navigator import STATUS_ABORTED, STATUS_SUCCEEDED, GoalCallbacks, GoalTicket, NavGoal

FOOTPRINT = [(0.21, 0.13), (0.21, -0.13), (-0.11, -0.13), (-0.11, 0.13)]
RES = 0.05


def bay_map() -> OccupancyMap:
    """4 m x 2 m room. A bay at the west end: 0.40 m wide (y 0.80..1.20), walls on three sides, open to the east."""
    w, h = 80, 40
    blocked = [False] * (w * h)

    def wall(x0, y0, x1, y1):
        for j in range(int(y0 / RES), int(math.ceil(y1 / RES))):
            for i in range(int(x0 / RES), int(math.ceil(x1 / RES))):
                blocked[j * w + i] = True

    wall(0, 0, 4, 0.05)
    wall(0, 1.95, 4, 2)
    wall(0, 0, 0.05, 2)
    wall(3.95, 0, 4, 2)
    wall(0.05, 0.05, 0.8, 0.75)   # bay side, south
    wall(0.05, 1.25, 0.8, 1.95)   # bay side, north
    return OccupancyMap(w, h, RES, (0.0, 0.0), blocked)


@pytest.fixture
def geometry():
    return DockingGeometry(bay_map(), FOOTPRINT, padding=0.02)


def test_open_floor_needs_no_docking(geometry):
    plan = geometry.plan((2.0, 1.0, 0.0), (3.0, 1.0, math.pi / 2))
    assert plan.plain


def test_facing_out_of_the_bay_turns_outside_and_backs_in(geometry):
    goal = (0.30, 1.0, 0.0)  # back to the west wall, nose to the opening
    assert not geometry.can_turn(goal[0], goal[1])
    plan = geometry.plan((2.5, 1.0, math.pi), goal)
    assert plan.approach is not None and plan.run_in is not None
    ax, ay, ayaw = plan.approach
    assert ay == pytest.approx(1.0) and ayaw == pytest.approx(0.0) and ax > goal[0]
    assert geometry.can_turn(ax, ay)
    assert plan.run_in.reverse  # backs onto the endpoint
    assert plan.run_in.end == pytest.approx(goal[:2])


def test_facing_into_the_bay_turns_outside_and_drives_in(geometry):
    goal = (0.30, 1.0, math.pi)  # nose to the west wall
    plan = geometry.plan((2.5, 1.0, 0.0), goal)
    assert plan.run_in is not None and not plan.run_in.reverse
    assert plan.approach[0] > goal[0]


def test_leaving_the_bay_drives_straight_out_first(geometry):
    start = (0.30, 1.0, math.pi)  # nose in: out backwards
    plan = geometry.plan(start, (3.0, 1.0, 0.0))
    assert plan.exit is not None and plan.exit.reverse
    assert geometry.can_turn(*plan.exit.end)
    start = (0.30, 1.0, 0.0)  # nose out: out forwards
    assert not geometry.plan(start, (3.0, 1.0, 0.0)).exit.reverse


def test_on_the_bay_axis_a_refine_is_only_the_straight_move(geometry):
    goal = (0.30, 1.0, 0.0)
    plan = geometry.plan((0.36, 1.01, 0.02), goal)
    assert plan.exit is None and plan.approach is None
    assert plan.run_in.reverse and plan.run_in.length == pytest.approx(0.0608, abs=1e-3)


def test_a_dead_end_with_no_turning_point_falls_back_to_the_plain_goal():
    w, h = 40, 40
    blocked = [True] * (w * h)
    for j in range(16, 24):  # a 0.40 m corridor, too narrow to turn anywhere
        for i in range(1, 39):
            blocked[j * w + i] = False
    geom = DockingGeometry(OccupancyMap(w, h, RES, (0.0, 0.0), blocked), FOOTPRINT, padding=0.02)
    plan = geom.plan((1.0, 1.0, 0.0), (0.5, 1.0, math.pi / 2))
    assert plan.approach is None and plan.run_in is None


def test_v006_bundle_bays():
    here = os.path.dirname(os.path.abspath(__file__))
    yaml = os.path.join(here, "..", "..", "warotrans_navigation", "maps", "map_one_v006", "map.yaml")
    if not os.path.isfile(yaml):
        pytest.skip("MAP-ONE-V006 bundle not in this checkout")
    geom = DockingGeometry(OccupancyMap.load(yaml), FOOTPRINT, padding=0.02)
    # EP-SHELF-C-01 (chassis centre) -> base_footprint 0.05 m behind
    goal = (-0.953 - 0.05, -0.111, 0.0)
    plan = geom.plan((0.15, 0.6, 0.0), goal)
    assert plan.run_in is not None and plan.run_in.reverse
    assert geom.can_turn(*plan.approach[:2])


# ---------------------------------------------------------------- navigator


class FakeFollow:
    def __init__(self):
        self.goals = []
        self.cancels = []

    def is_ready(self):
        return True

    def send_path(self, command_id, frame_id, poses, controller_id, ticket, callbacks):
        self.goals.append((list(poses), controller_id, ticket, callbacks))

    def cancel_goal(self, ticket, on_response):
        self.cancels.append(ticket)
        on_response(True, "ERROR_NONE")


class Sink:
    def __init__(self):
        self.responses, self.results, self.feedback = [], [], []

    def callbacks(self):
        return GoalCallbacks(
            on_response=lambda a, e: self.responses.append((a, e)),
            on_feedback=lambda f: self.feedback.append(f),
            on_result=lambda r: self.results.append(r),
        )


class Where:
    """Mutable robot pose for the navigator (tests move it as the stages finish)."""

    def __init__(self, pose):
        self.pose = pose

    def __call__(self):
        return self.pose


def _nav(geometry, start):
    inner, follow, proposals = FakeNavigator(), FakeFollow(), []
    where = start if isinstance(start, Where) else Where(start)
    nav = DockingNavigator(inner=inner, follow=follow, geometry=geometry, pose_provider=where,
                           propose_path=proposals.append)
    return nav, inner, follow, proposals


def _at(where, goal):
    where.pose = (goal.x, goal.y, goal.yaw)


def _ok(cb):
    from warotrans_fleet.navigator import NavResult

    cb.on_response(True, None)
    cb.on_result(NavResult(status=STATUS_SUCCEEDED))


def test_exit_travel_run_in_run_as_one_goal(geometry):
    where = Where((0.30, 1.0, math.pi))  # nose in, in the bay
    nav, inner, follow, proposals = _nav(geometry, where)
    sink = Sink()
    nav.send_goal(NavGoal("c1", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())  # same bay, nose out
    # exit: straight out backwards, proposed to traffic first
    assert len(follow.goals) == 1 and follow.goals[0][1] == CONTROLLER_REVERSE and len(proposals) == 1
    _ok(follow.goals[0][3])
    assert sink.responses == [(True, None)] and sink.results == []
    # travel: Nav2 to the turning point at the endpoint heading
    assert len(inner.goals) == 1 and inner.goals[0][0].yaw == pytest.approx(0.0)
    _at(where, inner.goals[0][0])
    _ok(inner.goals[0][2])
    # run in: backwards onto the endpoint
    assert len(follow.goals) == 2 and follow.goals[1][1] == CONTROLLER_REVERSE
    where.pose = (0.30, 1.0, 0.0)  # on the endpoint, right heading: the align step has nothing to do
    _ok(follow.goals[1][3])
    assert len(sink.results) == 1 and sink.results[0].status == STATUS_SUCCEEDED
    assert sink.responses == [(True, None)]


def test_a_failed_stage_ends_the_goal_and_no_later_stage_starts(geometry):
    from warotrans_fleet.navigator import NavResult

    nav, inner, follow, _ = _nav(geometry, (2.5, 1.0, math.pi))
    sink = Sink()
    nav.send_goal(NavGoal("c2", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    cb = inner.goals[0][2]
    cb.on_response(True, None)
    cb.on_result(NavResult(status=STATUS_ABORTED, error_code=103))
    assert sink.results[0].status == STATUS_ABORTED and follow.goals == []


def test_cancel_stops_the_running_stage(geometry):
    where = Where((2.5, 1.0, math.pi))
    nav, inner, follow, _ = _nav(geometry, where)
    sink, ticket = Sink(), GoalTicket()
    nav.send_goal(NavGoal("c3", "map", 0.30, 1.0, 0.0), ticket, sink.callbacks())
    _at(where, inner.goals[0][0])
    _ok(inner.goals[0][2])  # travel done, run in running
    answers = []
    nav.cancel_goal(ticket, lambda a, d: answers.append(a))
    assert follow.cancels and answers == [True]


def test_plain_goal_goes_straight_to_nav2(geometry):
    nav, inner, follow, _ = _nav(geometry, (2.0, 1.0, 0.0))
    ticket = GoalTicket()
    nav.send_goal(NavGoal("c4", "map", 3.0, 1.0, 1.57), ticket, Sink().callbacks())
    assert len(inner.goals) == 1 and inner.goals[0][0].x == 3.0 and follow.goals == []


def test_forward_controller_for_a_forward_run_in(geometry):
    where = Where((2.5, 1.0, 0.0))
    nav, inner, follow, _ = _nav(geometry, where)
    nav.send_goal(NavGoal("c5", "map", 0.30, 1.0, math.pi), GoalTicket(), Sink().callbacks())
    _at(where, inner.goals[0][0])
    _ok(inner.goals[0][2])
    assert follow.goals[0][1] == CONTROLLER_FORWARD


def test_run_in_is_skipped_when_the_robot_already_stands_on_the_endpoint(geometry):
    where = Where((2.5, 1.0, math.pi))
    nav, inner, follow, _ = _nav(geometry, where)
    sink = Sink()
    nav.send_goal(NavGoal("c6", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    where.pose = (0.33, 1.0, 0.0)  # Nav2 stopped 3 cm from the endpoint
    _ok(inner.goals[0][2])
    assert follow.goals == [] and sink.results[0].status == STATUS_SUCCEEDED


def test_the_goals_position_tolerance_decides_when_the_robot_is_on_the_endpoint(geometry):
    """payload.positionTolerance (the target Endpoint's, 2026-11-13: 0.12 m) replaces docking_arrive_xy_m (0.04 here)."""
    where = Where((2.5, 1.0, math.pi))
    nav, inner, follow, _ = _nav(geometry, where)
    sink = Sink()
    nav.send_goal(NavGoal("c6b", "map", 0.30, 1.0, 0.0, position_tolerance=0.12), GoalTicket(), sink.callbacks())
    where.pose = (0.40, 1.0, 0.0)  # Nav2 stopped 10 cm short: inside 0.12, no run in, no align
    _ok(inner.goals[0][2])
    assert follow.goals == [] and len(inner.goals) == 1 and sink.results[0].status == STATUS_SUCCEEDED


def test_without_a_position_tolerance_the_configured_arrive_distance_applies(geometry):
    where = Where((2.5, 1.0, math.pi))
    nav, inner, follow, _ = _nav(geometry, where)
    sink = Sink()
    nav.send_goal(NavGoal("c6c", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    where.pose = (0.40, 1.0, 0.0)  # 10 cm short of the endpoint, more than 0.04: run in
    _ok(inner.goals[0][2])
    assert len(follow.goals) == 1 and sink.results == []


def test_run_in_starts_from_where_the_robot_stopped(geometry):
    where = Where((2.5, 1.0, math.pi))
    nav, inner, follow, _ = _nav(geometry, where)
    nav.send_goal(NavGoal("c7", "map", 0.30, 1.0, 0.0), GoalTicket(), Sink().callbacks())
    where.pose = (0.52, 1.02, 0.01)
    _ok(inner.goals[0][2])
    poses = follow.goals[0][0]
    assert poses[0][:2] == pytest.approx((0.52, 1.02)) and poses[-1][:2] == pytest.approx((0.30, 1.0))
    assert follow.goals[0][1] == CONTROLLER_REVERSE


def test_straight_move_poses_are_dense():
    m = StraightMove((0.0, 0.0), (0.12, 0.0), 0.0, False)
    poses = m.poses(0.05)
    assert poses[0] == (0.0, 0.0, 0.0) and poses[-1][0] == pytest.approx(0.12) and len(poses) == 4


def test_a_run_in_that_stalls_close_to_the_wall_goes_on_to_align(geometry):
    from warotrans_fleet.navigator import NavResult

    where = Where((2.5, 1.0, math.pi))
    nav, inner, follow, _ = _nav(geometry, where)
    sink = Sink()
    nav.send_goal(NavGoal("c8", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    _at(where, inner.goals[0][0])
    _ok(inner.goals[0][2])
    where.pose = (0.32, 1.0, 0.12)  # 2 cm short, 7 degrees off: "failed to make progress"
    cb = follow.goals[0][3]
    cb.on_response(True, None)
    cb.on_result(NavResult(status=STATUS_ABORTED))
    assert sink.results == [] and len(inner.goals) == 2  # align: NavigateToPose onto the endpoint itself
    assert (inner.goals[1][0].x, inner.goals[1][0].y, inner.goals[1][0].yaw) == (0.30, 1.0, 0.0)
    _ok(inner.goals[1][2])
    assert sink.results[0].status == STATUS_SUCCEEDED


def _nav_opts(geometry, start, **options):
    inner, follow = FakeNavigator(), FakeFollow()
    where = start if isinstance(start, Where) else Where(start)
    nav = DockingNavigator(inner=inner, follow=follow, geometry=geometry, pose_provider=where, **options)
    return nav, inner, follow, where


def _fail(cb, status=STATUS_ABORTED):
    from warotrans_fleet.navigator import NavResult

    cb.on_response(True, None)
    cb.on_result(NavResult(status=status))


class Driver:
    """Completes the stages the navigator starts, one at a time, moving the robot to where each one ends."""

    def __init__(self, inner, follow, where):
        self.inner, self.follow, self.where = inner, follow, where
        self.seen_inner, self.seen_follow = len(inner.goals), len(follow.goals)
        self.kinds = []

    def pending(self):
        if len(self.inner.goals) > self.seen_inner:
            return "nav", self.inner.goals[-1]
        if len(self.follow.goals) > self.seen_follow:
            return "path", self.follow.goals[-1]
        return None

    def step(self, ok=True):
        kind, goal = self.pending()
        if kind == "nav":
            self.seen_inner = len(self.inner.goals)
            g, cb = goal[0], goal[2]
            if ok:
                self.where.pose = (g.x, g.y, g.yaw)
        else:
            self.seen_follow = len(self.follow.goals)
            cb = goal[3]
            if ok:
                self.where.pose = goal[0][-1]
        self.kinds.append(kind)
        (_ok if ok else _fail)(cb)
        return kind

    def run(self, limit=12):
        while self.pending() is not None and limit:
            self.step()
            limit -= 1


def test_a_run_in_that_stops_far_from_the_endpoint_docks_again(geometry):
    nav, inner, follow, where = _nav_opts(geometry, (2.5, 1.0, math.pi))
    sink = Sink()
    nav.send_goal(NavGoal("c9", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    _at(where, inner.goals[0][0])
    _ok(inner.goals[0][2])
    stale = follow.goals[0][3]
    d = Driver(inner, follow, where)
    _fail(stale)  # blocked at the turning point: turn / come in again, not the end of the goal
    assert sink.results == [] and d.pending() is not None
    _fail(stale)  # a late duplicate of the failed stage changes nothing
    assert sink.results == []
    d.run()
    assert len(sink.results) == 1 and sink.results[0].status == STATUS_SUCCEEDED
    assert where.pose[:2] == pytest.approx((0.30, 1.0))
    assert "path" in d.kinds  # it came in straight again


def test_a_failed_align_docks_again(geometry):
    nav, inner, follow, where = _nav_opts(geometry, (2.5, 1.0, math.pi))
    sink = Sink()
    d = Driver(inner, follow, where)
    nav.send_goal(NavGoal("c10", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    d.step()  # travel
    kind, goal = d.pending()
    assert kind == "path"
    d.seen_follow = len(follow.goals)
    where.pose = (0.30, 1.0, 0.25)  # on the endpoint, 14 degrees off
    _ok(goal[3])
    d.step(ok=False)  # align: the wall stops the in-place turn
    assert sink.results == []
    # the new plan leaves the bay first: a straight move out, not another turn on the spot
    assert d.pending() is not None and d.pending()[0] == "path"
    d.run()
    assert len(sink.results) == 1 and sink.results[0].status == STATUS_SUCCEEDED


def test_a_plain_goal_that_fails_at_the_endpoint_docks_instead(geometry):
    nav, inner, follow, where = _nav_opts(geometry, (2.0, 1.0, 0.0))
    sink = Sink()
    nav.send_goal(NavGoal("c11", "map", 3.0, 1.0, math.pi / 2), GoalTicket(), sink.callbacks())
    assert len(inner.goals) == 1 and follow.goals == []  # the map says it can turn there: a plain goal
    where.pose = (2.92, 1.0, 0.3)  # Nav2 gave up turning 8 cm from it
    d = Driver(inner, follow, where)
    _fail(inner.goals[0][2])
    assert sink.results == [] and d.pending() is not None
    d.run()
    assert len(sink.results) == 1 and sink.results[0].status == STATUS_SUCCEEDED
    assert "path" in d.kinds  # straight onto the endpoint at its heading


def test_a_plain_goal_that_fails_far_from_the_endpoint_just_fails(geometry):
    nav, inner, follow, where = _nav_opts(geometry, (2.0, 1.0, 0.0))
    sink = Sink()
    nav.send_goal(NavGoal("c12", "map", 3.0, 1.0, math.pi / 2), GoalTicket(), sink.callbacks())
    _fail(inner.goals[0][2])  # still 1 m away
    assert sink.results[0].status == STATUS_ABORTED and len(inner.goals) == 1 and follow.goals == []


def test_redocks_are_limited(geometry):
    nav, inner, follow, where = _nav_opts(geometry, (2.5, 1.0, math.pi), max_redocks=1)
    sink = Sink()
    d = Driver(inner, follow, where)
    nav.send_goal(NavGoal("c13", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    d.step()  # travel
    d.step(ok=False)  # run in fails where it started: redock 1/1
    assert sink.results == []
    # drive the new plan up to its run in, and fail that one too
    while d.pending() is not None:
        kind, goal = d.pending()
        is_run_in = kind == "path" and goal[0][-1][:2] == pytest.approx((0.30, 1.0))
        d.step(ok=not is_run_in)
        if is_run_in:
            break
    assert len(sink.results) == 1 and sink.results[0].status == STATUS_ABORTED
    assert d.pending() is None


def test_redock_disabled_keeps_the_old_behaviour(geometry):
    nav, inner, follow, where = _nav_opts(geometry, (2.5, 1.0, math.pi), max_redocks=0)
    sink = Sink()
    nav.send_goal(NavGoal("c14", "map", 0.30, 1.0, 0.0), GoalTicket(), sink.callbacks())
    _at(where, inner.goals[0][0])
    _ok(inner.goals[0][2])
    _fail(follow.goals[0][3])
    assert sink.results[0].status == STATUS_ABORTED and len(inner.goals) == 1


def test_cancel_during_a_redock_stops_it(geometry):
    from warotrans_fleet.navigator import STATUS_CANCELED

    nav, inner, follow, where = _nav_opts(geometry, (2.5, 1.0, math.pi))
    sink, ticket = Sink(), GoalTicket()
    d = Driver(inner, follow, where)
    nav.send_goal(NavGoal("c15", "map", 0.30, 1.0, 0.0), ticket, sink.callbacks())
    d.step()
    d.step(ok=False)  # redock starts
    kind, goal = d.pending()
    nav.cancel_goal(ticket, lambda a, _d: None)
    assert (inner.cancel_requests if kind == "nav" else follow.cancels)
    before = len(inner.goals) + len(follow.goals)
    _fail(goal[2] if kind == "nav" else goal[3], status=STATUS_CANCELED)
    assert len(sink.results) == 1 and sink.results[0].status == STATUS_CANCELED
    assert len(inner.goals) + len(follow.goals) == before  # nothing after the cancel


def test_a_failed_exit_does_not_back_out_again(geometry):
    nav, inner, follow, where = _nav_opts(geometry, (0.30, 1.0, math.pi))  # nose in
    sink = Sink()
    nav.send_goal(NavGoal("c16", "map", 3.0, 1.0, 0.0), GoalTicket(), sink.callbacks())
    assert follow.goals[0][1] == CONTROLLER_REVERSE  # backing out
    _fail(follow.goals[0][3])  # something behind the robot
    # forward is the bay wall: no second reverse exit; the goal fails or travels, it does not ram the same thing
    assert all(g[1] != CONTROLLER_REVERSE for g in follow.goals[1:])


def test_force_dock_plans_a_straight_run_in_with_a_wider_turning_margin(geometry):
    goal = (3.0, 1.0, math.pi / 2)
    assert geometry.plan((2.9, 1.0, 0.3), goal).plain
    plan = geometry.plan((2.9, 1.0, 0.3), goal, force_dock=True, refine=False)
    assert plan.run_in is not None and plan.run_in.end == pytest.approx(goal[:2])
    assert geometry.can_turn(*plan.approach[:2], strict=True)
