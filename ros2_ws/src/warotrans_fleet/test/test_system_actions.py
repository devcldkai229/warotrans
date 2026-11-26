"""SYSTEM_ACTION: only announced actions, only while held and idle, a failed restart is reported, detached ones answer first."""

import logging

from fleet_fakes import ROBOT, Recorder

from warotrans_fleet.command_router import CommandRouter
from warotrans_fleet.payload_builder import build_heartbeat
from warotrans_fleet.system_actions import (
    REBOOT,
    RESTART_LOCALIZATION,
    RESTART_NAV,
    SystemAction,
    SystemActionHandler,
    parse_action_list,
)


def _body(command_id: str, action: str | None) -> dict:
    return {
        "schemaVersion": 1,
        "messageId": "m",
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": "SYSTEM_ACTION",
        "payload": {} if action is None else {"action": action},
    }


def _setup(actions, *, held=True, busy=False):
    rec = Recorder()
    router = CommandRouter(robot_code=ROBOT, publisher=rec, log=logging.getLogger("t"))
    state = {"held": held, "busy": busy}
    handler = SystemActionHandler(
        actions,
        motion_held=lambda: state["held"],
        busy=lambda: state["busy"],
        log=logging.getLogger("t"),
        run_async=lambda fn: fn(),  # synchronous for the test
    )
    router.register(handler)
    return rec, router, handler, state


def test_parse_action_list_keeps_known_names_once_in_order():
    assert parse_action_list(" restart_nav, REBOOT ,bogus, RESTART_NAV") == ["RESTART_NAV", "REBOOT"]
    assert parse_action_list("") == []


def test_announced_lists_only_what_this_robot_can_run_in_a_stable_order():
    _, _, handler, _ = _setup({REBOOT: SystemAction(run=lambda: None), RESTART_NAV: SystemAction(run=lambda: None)})
    assert handler.announced == ["RESTART_NAV", "REBOOT"]


def test_restart_nav_runs_and_succeeds_while_held_and_idle():
    calls = []
    rec, router, _, _ = _setup({RESTART_NAV: SystemAction(run=lambda: calls.append("nav"))})
    router.handle_message(_body("c1", "RESTART_NAV"))
    assert calls == ["nav"]
    assert rec.acks[-1]["accepted"] is True
    assert rec.results[-1]["outcome"] == "SUCCEEDED"
    assert rec.results[-1]["result"] == {"action": "RESTART_NAV"}


def test_a_failed_restart_is_reported_failed_with_the_reason():
    rec, router, _, _ = _setup({RESTART_LOCALIZATION: SystemAction(run=lambda: "manage_nodes is not available")})
    router.handle_message(_body("c1", "RESTART_LOCALIZATION"))
    result = rec.results[-1]
    assert result["outcome"] == "FAILED"
    assert result["errorCode"] == "SYSTEM_ACTION_FAILED"
    assert "not available" in result["errorMessage"]


def test_refused_unless_the_robot_holds_motion():
    calls = []
    rec, router, _, _ = _setup({RESTART_NAV: SystemAction(run=lambda: calls.append(1))}, held=False)
    router.handle_message(_body("c1", "RESTART_NAV"))
    assert calls == []
    assert rec.acks[-1]["accepted"] is False
    assert rec.acks[-1]["reasonCode"] == "MOTION_NOT_HELD"


def test_refused_while_a_move_runs():
    rec, router, _, _ = _setup({RESTART_NAV: SystemAction(run=lambda: None)}, busy=True)
    router.handle_message(_body("c1", "RESTART_NAV"))
    assert rec.acks[-1]["accepted"] is False
    assert rec.acks[-1]["reasonCode"] == "BUSY"


def test_an_action_this_robot_does_not_offer_is_refused():
    rec, router, _, _ = _setup({RESTART_NAV: SystemAction(run=lambda: None)})
    router.handle_message(_body("c1", "SHUTDOWN"))
    assert rec.acks[-1]["accepted"] is False
    assert rec.acks[-1]["reasonCode"] == "UNSUPPORTED_COMMAND"


def test_reboot_without_a_sudo_rule_is_refused_before_anything_runs():
    calls = []
    rec, router, _, _ = _setup({REBOOT: SystemAction(run=lambda: calls.append(1), precheck=lambda: "reboot needs a password")})
    router.handle_message(_body("c1", "REBOOT"))
    assert calls == []
    assert rec.acks[-1]["accepted"] is False
    assert rec.acks[-1]["reasonCode"] == "ACTION_NOT_PERMITTED"


def test_reboot_answers_succeeded_before_it_runs():
    order = []

    rec, router, _, _ = _setup({})

    def run():
        order.append(("run", len(rec.results)))
        return None

    router._handlers["SYSTEM_ACTION"]._actions[REBOOT] = SystemAction(run=run)
    router.handle_message(_body("c1", "REBOOT"))
    assert rec.results[-1]["outcome"] == "SUCCEEDED"
    assert rec.results[-1]["result"] == {"action": "REBOOT", "scheduled": True}
    # The result had already been published when the reboot ran.
    assert order == [("run", 1)]


def test_heartbeat_carries_capabilities_only_when_given():
    assert "capabilities" not in build_heartbeat(robot_code=ROBOT, boot_id="b", sequence=1)
    body = build_heartbeat(robot_code=ROBOT, boot_id="b", sequence=1, capabilities=["RESTART_NAV"])
    assert body["capabilities"] == ["RESTART_NAV"]
