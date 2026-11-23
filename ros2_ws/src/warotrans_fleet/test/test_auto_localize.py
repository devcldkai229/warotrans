"""AUTO_LOCALIZE: takes a pose only when sure, spins only when allowed, always answers with what it saw."""

import logging
import math

from fleet_fakes import ROBOT, Recorder
from test_scan_match_localizer import ASYMMETRIC, SYMMETRIC, _scan

from warotrans_fleet.auto_localize import AutoLocalizeCommandHandler
from warotrans_fleet.command_router import CommandRouter
from warotrans_fleet.scan_match_localizer import LikelihoodField

LOG = logging.getLogger("t")


def _body(command_id: str, payload: dict) -> dict:
    return {
        "schemaVersion": 1,
        "messageId": "m",
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": "AUTO_LOCALIZE",
        "payload": payload,
    }


class Robot:
    """The ROS side the handler talks to, recorded."""

    def __init__(self, grid, true_pose, converges=None, spin_pose=None):
        self.field = LikelihoodField(grid)
        self.grid = grid
        self.true_pose = true_pose
        self.published: list[tuple[float, float, float]] = []
        self.converges = converges  # None = converges
        self.spins = 0
        self.spin_pose = spin_pose
        self.busy = False

    def points(self, _timeout):
        return _scan(self.grid, *self.true_pose)

    def spin(self):
        self.spins += 1
        return None

    def pose(self):
        return self.spin_pose


def _setup(robot: Robot, with_spin: bool = True):
    rec = Recorder()
    router = CommandRouter(robot_code=ROBOT, publisher=rec, log=LOG)
    handler = AutoLocalizeCommandHandler(
        busy=lambda: robot.busy,
        get_points=robot.points,
        get_field=lambda: robot.field,
        publish_initial_pose=lambda x, y, yaw: robot.published.append((x, y, yaw)),
        wait_converged=lambda _t: robot.converges,
        log=LOG,
        global_localize=robot.spin if with_spin else None,
        current_pose=robot.pose,
        run_async=lambda fn: fn(),  # run inline: the result is there when handle_message returns
    )
    router.register(handler)
    return rec, router


HOME = {"x": 2.45, "y": 0.7, "yaw": 0.0, "endpointCode": "EP-PARKING-A", "source": "HOME"}


def test_a_sure_match_is_published_and_succeeds():
    robot = Robot(ASYMMETRIC, (2.6, 0.6, 1.2))
    rec, router = _setup(robot)

    router.handle_message(_body("a-1", {"candidates": [HOME]}))

    assert rec.acks[0]["accepted"] is True
    result = rec.results[0]
    assert result["outcome"] == "SUCCEEDED"
    detail = result["result"]
    assert detail["method"] == "CANDIDATE_MATCH"
    assert detail["endpointCode"] == "EP-PARKING-A"
    assert math.hypot(detail["x"] - 2.6, detail["y"] - 0.6) < 0.06
    assert len(robot.published) == 1
    assert robot.spins == 0


def test_a_look_alike_place_publishes_nothing_and_returns_suggestions():
    robot = Robot(SYMMETRIC, (2.0, 1.0, 0.0))
    rec, router = _setup(robot)

    router.handle_message(_body("a-2", {"candidates": [{"x": 2.0, "y": 1.0, "endpointCode": "EP-MID"}]}))

    result = rec.results[0]
    assert result["outcome"] == "FAILED"
    assert result["errorCode"] == "LOCALIZATION_AMBIGUOUS"
    assert result["result"]["method"] == "NONE"
    assert len(result["result"]["suggestions"]) >= 2
    assert robot.published == []
    assert robot.spins == 0  # not allowed to move


def test_spin_only_when_allowed_and_the_spun_pose_is_checked_against_the_scan():
    robot = Robot(ASYMMETRIC, (0.5, 2.3, 0.0), spin_pose=(0.5, 2.3, 0.0))
    rec, router = _setup(robot)

    router.handle_message(_body("a-3", {"candidates": [{"x": 3.6, "y": 0.5}], "allowSpin": True}))

    result = rec.results[0]
    assert robot.spins == 1
    assert result["outcome"] == "SUCCEEDED"
    assert result["result"]["method"] == "GLOBAL_SPIN"


def test_a_spin_that_ends_in_the_wrong_place_is_refused():
    robot = Robot(ASYMMETRIC, (0.5, 2.3, 0.0), spin_pose=(3.5, 0.7, math.pi))
    rec, router = _setup(robot)

    router.handle_message(_body("a-4", {"candidates": [{"x": 3.6, "y": 0.5}], "allowSpin": True}))

    assert rec.results[0]["outcome"] == "FAILED"
    assert rec.results[0]["errorCode"] == "LOCALIZATION_NO_MATCH"


def test_no_spin_support_means_suggestions_only():
    robot = Robot(ASYMMETRIC, (0.5, 2.3, 0.0))
    rec, router = _setup(robot, with_spin=False)

    router.handle_message(_body("a-5", {"candidates": [{"x": 3.6, "y": 0.5}], "allowSpin": True}))

    result = rec.results[0]
    assert result["errorCode"] == "LOCALIZATION_NO_MATCH"
    # The whole-map check still says where the scan fits best.
    best = result["result"]["suggestions"][0]
    assert best["source"] == "GLOBAL" and math.hypot(best["x"] - 0.5, best["y"] - 2.3) < 0.08


def test_a_localizer_that_does_not_converge_fails_the_command():
    robot = Robot(ASYMMETRIC, (2.6, 0.6, 1.2), converges="AMCL_COVARIANCE_HIGH")
    rec, router = _setup(robot)

    router.handle_message(_body("a-6", {"candidates": [HOME]}))

    assert rec.results[0]["errorCode"] == "LOCALIZATION_NOT_CONVERGED"
    assert rec.results[0]["result"]["detail"] == "AMCL_COVARIANCE_HIGH"


def test_refused_while_a_move_runs_or_without_candidates():
    robot = Robot(ASYMMETRIC, (2.6, 0.6, 1.2))
    rec, router = _setup(robot)
    robot.busy = True
    router.handle_message(_body("a-7", {"candidates": [HOME]}))
    assert rec.acks[-1]["accepted"] is False
    assert rec.acks[-1]["reasonCode"] == "BUSY"

    robot.busy = False
    router.handle_message(_body("a-8", {"candidates": []}))
    assert rec.acks[-1]["accepted"] is False
    assert rec.acks[-1]["reasonCode"] == "INVALID_PAYLOAD"
