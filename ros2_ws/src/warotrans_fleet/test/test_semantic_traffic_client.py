"""semantic_active client: the robot moves only inside what the backend reserved for its current plan."""

from __future__ import annotations

import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

from warotrans_fleet import semantic_traffic_client as stc
from warotrans_fleet.semantic_traffic_client import SemanticTrafficClient

VECTORS = json.loads((Path(__file__).parent / "data" / "semantic_cells_vectors.json").read_text(encoding="utf-8"))
GEOMETRY = VECTORS["geometry"]
ZONES = VECTORS["zones"]
CMD = "11111111-1111-1111-1111-111111111111"
PATH = [(x / 10.0, 0.0, 0.0) for x in range(0, 31)]  # 0 → 3 m along x


class Clock:
    def __init__(self) -> None:
        self.wall = 1_800_000_000.0
        self.mono = 1000.0

    def advance(self, sec: float) -> None:
        self.wall += sec
        self.mono += sec


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat().replace("+00:00", "Z")


def make(clock: Clock | None = None):
    clock = clock or Clock()
    published: list[tuple[str, dict]] = []
    gate: list[bool] = []
    client = SemanticTrafficClient(
        robot_code="SIM-A",
        topic_prefix="warotrans-e2e/v1",
        fingerprint="fp-1",
        publish_json=lambda t, b: published.append((t, b)),
        set_authority_ok=gate.append,
        zones=ZONES,
        now=lambda: clock.wall,
        monotonic=lambda: clock.mono,
        authority_loss_timeout_sec=3.0,
    )
    return client, published, gate, clock


def grant_for(client: SemanticTrafficClient, clock: Clock, *, rev=None, plan_hash=None, epoch=100, seq=1,
              authority="auth-1", resources=None, ttl=30.0, fingerprint="fp-1", geometry=GEOMETRY) -> dict:
    geom = stc.SemanticGeometry.from_payload(GEOMETRY)
    return {
        "targetCommandId": CMD,
        "authorityId": authority,
        "epoch": epoch,
        "seq": seq,
        "semanticMapFingerprint": fingerprint,
        "planRevision": client.plan_revision if rev is None else rev,
        "planHash": client.plan_hash if plan_hash is None else plan_hash,
        "resourceIds": sorted(resources if resources is not None else stc.collect_resources(client.plan_poses, ZONES, geom)),
        "expiresAt": _iso(clock.wall + ttl),
        "geometry": geometry,
    }


def granted(clock: Clock | None = None):
    client, published, gate, clock = make(clock)
    client.on_plan(PATH, goal=PATH[-1], command_id=CMD)
    assert client.on_authority_payload(grant_for(client, clock), target_command_id=CMD)
    return client, published, gate, clock


# ---------------------------------------------------------------------- geometry parity with the backend


def test_cells_match_backend_vectors():
    geom = stc.SemanticGeometry.from_payload(GEOMETRY)
    for case in VECTORS["cases"]:
        got = sorted(stc.collect_resources(case["poses"], ZONES, geom))
        assert got == case["resources"], case["name"]


def test_footprint_cells_match_backend_vectors():
    for fp in VECTORS["footprints"]:
        got = sorted(stc.footprint_resources(fp["x"], fp["y"], fp["radius"], ZONES, GEOMETRY["cellSizeMeters"]))
        assert got == fp["resources"]


# ---------------------------------------------------------------------- proposing


def test_plan_without_backend_command_is_never_proposed():
    client, published, gate, _ = make()
    client.on_plan(PATH, command_id=None)  # RViz / CLI goal
    assert published == []
    assert client.tick((0.0, 0.0, 0.0)) is False


def test_plan_proposal_carries_revision_hash_and_command():
    client, published, _, _ = make()
    client.on_plan(PATH, goal=PATH[-1], command_id=CMD)
    topic, body = published[-1]
    assert topic == "warotrans-e2e/v1/robots/SIM-A/traffic_plan"
    assert body["planRevision"] == 1 and body["commandId"] == CMD and body["planHash"] == client.plan_hash


def test_identical_plan_is_not_re_proposed_while_granted():
    client, published, _, clock = granted()
    n = len(published)
    clock.advance(10)
    client.on_plan(PATH, goal=PATH[-1], command_id=CMD)
    assert len(published) == n
    assert client.plan_revision == 1


# ---------------------------------------------------------------------- A: grant → renew → exit


def test_grant_opens_the_gate_inside_the_reservation_and_renew_keeps_it_open():
    client, _, gate, clock = granted()
    assert client.tick((0.0, 0.0, 0.0)) is True and gate[-1] is True
    for _ in range(5):
        clock.advance(1.5)
        assert client.on_renew({"authorityId": "auth-1", "epoch": 100, "seq": client.grant.seq + 1,
                                "expiresAt": _iso(clock.wall + 30)})
        assert client.tick((0.5, 0.0, 0.0)) is True


def test_verified_exit_reports_only_resources_entered_and_left():
    client, _, _, clock = granted()
    assert client.resources_cleared_at(0.0, 0.0) == []
    assert client.resources_cleared_at(2.0, 0.0) == []  # pending until clear_confirm_sec
    clock.advance(client.clear_confirm_sec + 0.05)
    cleared = client.resources_cleared_at(2.0, 0.0)
    assert cleared and all(c in client.grant.resource_ids for c in cleared)
    near_end = stc.footprint_resources(2.0, 0.0, GEOMETRY["footprintRadiusMeters"] + 0.05, ZONES, 0.25)
    assert not set(cleared) & near_end


# ---------------------------------------------------------------------- staying inside the reservation


def test_gate_closes_when_the_robot_leaves_the_reserved_corridor():
    client, _, gate, _ = granted()
    assert client.tick((1.0, 0.0, 0.0)) is True
    assert client.tick((1.0, 0.6, 0.0)) is False  # drifted 0.6 m sideways
    assert client.last_block_reason == "OUTSIDE_RESERVATION" and gate[-1] is False


def test_gate_closes_beyond_the_reserved_horizon():
    client, published, _, clock = make()
    long_path = [(x / 2.0, 0.0, 0.0) for x in range(0, 21)]  # 10 m, horizon 4 m
    client.on_plan(long_path, command_id=CMD)
    assert client.on_authority_payload(grant_for(client, clock), target_command_id=CMD)
    assert client.tick((3.5, 0.0, 0.0)) is True
    assert client.tick((5.5, 0.0, 0.0)) is False
    assert client.last_block_reason == "OUTSIDE_RESERVATION"


def test_unknown_pose_closes_the_gate():
    client, _, _, _ = granted()
    assert client.tick(None) is False
    assert client.last_block_reason == "POSE_UNKNOWN"


# ---------------------------------------------------------------------- F: replanning


def test_replan_never_widens_where_the_robot_may_go():
    client, _, _, clock = granted()
    # New plan turns off the reserved corridor: until its own grant arrives only old∩new cells are usable.
    detour = [(1.0, y / 10.0, 1.57) for y in range(0, 21)]
    client.on_plan(detour, command_id=CMD)
    assert client.plan_revision == 2
    assert client.tick((1.0, 0.0, 1.57)) is True   # still on cells the backend holds for both paths
    assert client.tick((1.0, 0.8, 1.57)) is False  # new path, not reserved yet
    assert client.last_block_reason == "OUTSIDE_RESERVATION"
    # Grant for the new revision: now allowed.
    assert client.on_authority_payload(grant_for(client, clock, authority="auth-2", seq=2), target_command_id=CMD)
    assert client.tick((1.0, 0.8, 1.57)) is True


def test_grant_for_an_older_plan_revision_or_other_hash_is_refused():
    client, _, _, clock = granted()
    client.on_plan([(0.0, 0.0, 0.0), (0.0, 2.0, 1.57)], command_id=CMD)
    assert not client.on_authority_payload(grant_for(client, clock, rev=1, authority="late", seq=9), target_command_id=CMD)
    assert client.last_block_reason == "STALE_PLAN_REVISION"
    assert not client.on_authority_payload(grant_for(client, clock, plan_hash="other", authority="x", seq=9), target_command_id=CMD)
    assert client.last_block_reason == "PLAN_HASH_MISMATCH"


# ---------------------------------------------------------------------- J: duplicate / expired / delayed / revoked


def test_redelivered_grant_and_older_renew_change_nothing():
    client, _, _, clock = granted()
    seq = client.grant.seq
    assert not client.on_authority_payload(grant_for(client, clock), target_command_id=CMD)  # same id + seq
    assert client.last_block_reason == "DUPLICATE_OR_OLDER"
    assert not client.on_renew({"authorityId": "auth-1", "epoch": 100, "seq": seq, "expiresAt": _iso(clock.wall + 30)})
    assert not client.on_renew({"authorityId": "other", "epoch": 100, "seq": seq + 5, "expiresAt": _iso(clock.wall + 30)})
    assert client.grant.seq == seq


def test_expired_on_arrival_wrong_map_or_other_command_never_opens():
    client, _, gate, clock = make()
    client.on_plan(PATH, command_id=CMD)
    assert not client.on_authority_payload(grant_for(client, clock, ttl=-1), target_command_id=CMD)
    assert client.last_block_reason == "EXPIRED_ON_ARRIVAL"
    assert not client.on_authority_payload(grant_for(client, clock, fingerprint="fp-other"), target_command_id=CMD)
    assert not client.on_authority_payload(grant_for(client, clock), target_command_id="22222222-2222-2222-2222-222222222222")
    assert not client.on_authority_payload(grant_for(client, clock, geometry=None), target_command_id=CMD)
    assert client.tick((0.0, 0.0, 0.0)) is False and gate[-1] is False


def test_backend_silence_stops_the_robot_within_the_loss_timeout_whatever_expiresAt_says():
    client, _, _, clock = granted()
    clock.advance(2.9)
    assert client.tick((0.0, 0.0, 0.0)) is True
    clock.advance(0.2)
    assert client.tick((0.0, 0.0, 0.0)) is False
    assert client.last_block_reason == "AUTHORITY_SILENT"


def test_expires_at_reached_closes_the_gate_even_with_renews_heard():
    client, _, _, clock = make()
    client.on_plan(PATH, command_id=CMD)
    assert client.on_authority_payload(grant_for(client, clock, ttl=2.0), target_command_id=CMD)
    clock.advance(2.5)
    assert client.tick((0.0, 0.0, 0.0)) is False
    assert client.last_block_reason in ("EXPIRED", "AUTHORITY_SILENT")


def test_revoke_closes_now_but_a_revoke_for_an_older_authority_is_ignored():
    client, _, gate, _ = granted()
    client.on_revoke({"authorityId": "older"})
    assert client.tick((0.0, 0.0, 0.0)) is True
    client.on_revoke({"authorityId": "auth-1"})
    assert gate[-1] is False
    assert client.tick((0.0, 0.0, 0.0)) is False
    assert client.grant is None


def test_link_loss_closes_the_gate():
    client, _, gate, _ = granted()
    client.on_link(False)
    assert gate[-1] is False
    assert client.tick((0.0, 0.0, 0.0)) is False and client.last_block_reason == "LINK_DOWN"
    client.on_link(True)
    assert client.tick((0.0, 0.0, 0.0)) is True


# ---------------------------------------------------------------------- K: backend restart / new epoch


def test_new_epoch_voids_older_grants_and_re_proposes():
    client, published, gate, clock = granted()
    n = len(published)
    client.on_epoch({"epoch": 200})
    assert gate[-1] is False and client.grant is None
    assert client.tick((0.0, 0.0, 0.0)) is False
    # A grant of the old epoch arriving late is refused.
    assert not client.on_authority_payload(grant_for(client, clock, epoch=100, authority="late"), target_command_id=CMD)
    assert client.last_block_reason == "STALE_EPOCH"
    client.on_plan(PATH, command_id=CMD)  # same path, cooldown reset by the epoch change
    assert len(published) == n + 1
    assert client.on_authority_payload(grant_for(client, clock, epoch=200, authority="new"), target_command_id=CMD)
    assert client.tick((0.0, 0.0, 0.0)) is True


def test_same_or_older_epoch_announcement_changes_nothing():
    client, _, _, _ = granted()
    client.on_epoch({"epoch": 100})
    client.on_epoch({"epoch": 50})
    assert client.tick((0.0, 0.0, 0.0)) is True


# ---------------------------------------------------------------------- command lifecycle


def test_finished_command_drops_its_grant():
    client, _, _, _ = granted()
    client.on_command_finished(CMD)
    assert client.grant is None and client.command_id is None
    assert client.tick((0.0, 0.0, 0.0)) is False


def test_held_for_traffic_reports_recent_gate_holds_of_the_command():
    client, _, _, clock = make()
    client.on_plan(PATH, command_id=CMD)
    client.tick((0.0, 0.0, 0.0))  # no grant yet
    assert client.held_for_traffic()
    clock.advance(10)
    assert not client.held_for_traffic()


def test_expires_at_with_dotnet_seven_digit_fraction_parses():
    ts = stc._parse_expires_at("2026-11-04T10:00:00.1234567+00:00")
    assert abs(ts - datetime(2026, 11, 4, 10, 0, 0, 123456, tzinfo=timezone.utc).timestamp()) < 1e-6
    assert stc._parse_expires_at(None) == 0.0
    assert stc._parse_expires_at("garbage") == 0.0
    _ = timedelta  # keep import for readability of ttl math above


def test_authority_commands_are_acked_with_the_refusal_reason():
    client, _, _, clock = make()
    client.on_plan(PATH, command_id=CMD)
    good = grant_for(client, clock)
    ack = client.handle_authority_command({"type": "EXTEND_AUTHORITY", "commandId": "c-1", "payload": good})
    assert ack["accepted"] is True and ack["commandId"] == "c-1" and ack["robotCode"] == "SIM-A"
    dup = client.handle_authority_command({"type": "EXTEND_AUTHORITY", "commandId": "c-2", "payload": good})
    assert dup["accepted"] is False and dup["reasonCode"] == "DUPLICATE_OR_OLDER"
    assert client.handle_authority_command({"type": "NAVIGATE_TO_POSE", "payload": {}}) is None
    revoke = client.handle_authority_command({"type": "REVOKE_AUTHORITY", "commandId": "c-3",
                                              "payload": {"authorityId": "auth-1"}})
    assert revoke["accepted"] is True and client.grant is None


def test_state_report_repeats_only_while_executing_without_a_grant():
    client, _, _, clock = make()
    assert client.state_report_due() is False  # no command
    client.on_plan(PATH, command_id=CMD)
    assert client.state_report_due() is True
    assert client.state_report_due() is False  # period
    clock.advance(5.1)
    assert client.state_report_due() is True
    assert client.on_authority_payload(grant_for(client, clock), target_command_id=CMD)
    clock.advance(2.6)
    assert client.on_renew({"authorityId": "auth-1", "epoch": 100, "seq": 2, "expiresAt": _iso(clock.wall + 30)})
    clock.advance(2.6)
    assert client.state_report_due() is False  # a working grant


def test_localization_regained_fires_once_on_the_rising_edge():
    client, _, _, _ = make()
    assert client.localization_regained(False) is False
    assert client.localization_regained(True) is True
    assert client.localization_regained(True) is False
    assert client.localization_regained(False) is False
    assert client.localization_regained(True) is True


# ---------------------------------------------------------------------- recovery inside the grant


def _at_x(x: float) -> tuple[float, float, float]:
    return (x, 0.0, 0.0)


def test_spin_needs_only_a_valid_position_in_the_grant():
    client, _, _, _ = granted()
    assert client.recovery_block_reason(_at_x(1.0), "spin") is None


def test_no_grant_no_recovery():
    client, _, _, _ = make()
    assert client.recovery_block_reason(_at_x(1.0), "spin") is not None
    assert client.recovery_block_reason(_at_x(1.0), "backup", 0.10) is not None


def test_no_pose_no_recovery():
    client, _, _, _ = granted()
    assert client.recovery_block_reason(None, "backup", 0.10) == "POSE_UNKNOWN"


def test_authority_silence_blocks_recovery():
    client, _, _, clock = granted()
    clock.advance(4.0)
    assert client.recovery_block_reason(_at_x(1.0), "backup", 0.10) == "AUTHORITY_SILENT"


def test_backup_inside_the_grant_is_allowed_at_the_path_start():
    # The reserve radius leaves ~0.13 m around every path sample, the path start included.
    client, _, _, _ = granted()
    assert client.recovery_block_reason(_at_x(0.0), "backup", 0.10) is None


def test_backup_past_the_grant_is_refused():
    client, _, _, _ = granted()
    assert client.recovery_block_reason(_at_x(0.0), "backup", 1.0) == "REVERSE_OUTSIDE_RESERVATION"


def test_backup_follows_the_robot_heading():
    client, _, _, _ = granted()
    # Facing -x, reversing goes along +x: down the granted path.
    assert client.recovery_block_reason((1.0, 0.0, math.pi), "backup", 0.5) is None
    # Facing +y, reversing goes to -y, away from the corridor.
    assert client.recovery_block_reason((1.0, 0.0, math.pi / 2), "backup", 1.0) == "REVERSE_OUTSIDE_RESERVATION"


def test_unknown_recovery_kind_refused():
    client, _, _, _ = granted()
    assert client.recovery_block_reason(_at_x(1.0), "strafe", 0.1) == "UNKNOWN_RECOVERY"
    assert client.recovery_block_reason(_at_x(1.0), "backup", -0.1) == "UNKNOWN_RECOVERY"


def _drive_until_something_is_released(client, clock=None, start=0.0, stop=3.0):
    x = start
    while x < stop:
        cleared = client.resources_cleared_at(x, 0.0)
        if not cleared and clock is not None and client._pending_clear:
            clock.advance(client.clear_confirm_sec + 0.05)
            cleared = client.resources_cleared_at(x, 0.0)
        if cleared:
            return x, cleared
        x += 0.02
    raise AssertionError("nothing was cleared")


def test_backup_into_what_the_robot_released_is_refused():
    client, _, _, clock = granted()
    x, cleared = _drive_until_something_is_released(client, clock)
    assert set(cleared) <= set(client.grant.resource_ids), "the grant still lists it: that is the trap"
    assert client.recovery_block_reason(_at_x(x), "backup", 0.12) == "REVERSE_OUTSIDE_RESERVATION"


def test_gate_closes_when_the_footprint_is_back_in_a_released_resource():
    client, _, gate, clock = granted()
    x, _ = _drive_until_something_is_released(client, clock)
    assert client.tick(_at_x(x)) is True
    assert client.tick(_at_x(x - 0.15)) is False
    assert client.last_block_reason == "IN_RELEASED_RESOURCE"
    assert gate[-1] is False


def test_jitter_at_the_release_boundary_does_not_shut_the_gate():
    client, _, _, clock = granted()
    x, _ = _drive_until_something_is_released(client, clock)
    assert client.tick(_at_x(x - 0.02)) is True


def test_a_lost_first_grant_is_proposed_again_whatever_dropped_the_old_one():
    """The reason text is NEW_COMMAND here: matching on NO_GRANT left the robot waiting forever."""
    client, published, _, clock = make()
    client.on_plan(PATH, goal=PATH[-1], command_id=CMD)
    assert len(published) == 1
    client.tick(_at_x(0.0))
    assert len(published) == 1  # within the cooldown
    clock.advance(client.propose_retry_sec + 0.1)
    client.tick(_at_x(0.0))
    assert len(published) == 2 and published[-1][1]["planRevision"] == published[0][1]["planRevision"]


def test_no_proposal_while_the_link_is_down():
    client, published, _, clock = make()
    client.on_plan(PATH, goal=PATH[-1], command_id=CMD)
    client.on_link(False)
    clock.advance(client.propose_retry_sec + 0.1)
    client.tick(_at_x(0.0))
    assert len(published) == 1


def test_a_lapsed_grant_is_reported_so_the_backend_can_cancel_a_closed_command():
    client, _, _, clock = granted()
    assert client.state_report_due() is False
    clock.advance(client.authority_loss_timeout_sec + 0.1)
    assert client.state_report_due() is True


# ---------------------------------------------------------------------- lane rectangles


ORIENTED = json.loads((Path(__file__).parent / "data" / "semantic_oriented_vectors.json").read_text(encoding="utf-8"))
LANE_GEOMETRY = ORIENTED["geometry"]
ALONG_X = [(x / 10.0, 0.0, 0.0) for x in range(0, 31)]


def test_lane_rectangles_match_backend_vectors():
    geom = stc.SemanticGeometry.from_payload(LANE_GEOMETRY)
    assert geom.lane_half_width == 0.26
    for case in ORIENTED["cases"]:
        got = sorted(stc.collect_resources(case["poses"], ORIENTED["zones"], geom))
        assert got == case["resources"], case["name"]


def test_body_rectangles_match_backend_vectors():
    for b in ORIENTED["bodies"]:
        got: set[str] = set()
        stc.rect_cells(b["x"], b["y"], b["heading"], b["halfLength"], b["halfWidth"], LANE_GEOMETRY["cellSizeMeters"], got)
        assert sorted(got) == b["resources"]


def test_geometry_without_the_body_size_is_refused_with_lane_rectangles():
    assert stc.SemanticGeometry.from_payload({**LANE_GEOMETRY, "footprintHalfWidthMeters": 0}) is None


def _lane_grant():
    client, _, gate, clock = make()
    client.zones = []
    client.on_plan(ALONG_X, goal=ALONG_X[-1], command_id=CMD)
    geom = stc.SemanticGeometry.from_payload(LANE_GEOMETRY)
    assert client.on_authority_payload(
        grant_for(client, clock, geometry=LANE_GEOMETRY, resources=sorted(stc.collect_resources(ALONG_X, [], geom))),
        target_command_id=CMD,
    )
    return client, gate


def test_body_on_the_lane_keeps_the_gate_open():
    client, _ = _lane_grant()
    for dy in (0.0, 0.05, -0.05):
        assert client.tick((1.5, dy, 0.0)) is True, dy


def test_drifting_out_of_the_lane_rectangle_closes_the_gate():
    client, gate = _lane_grant()
    assert client.tick((1.5, 0.15, 0.0)) is False
    assert client.last_block_reason == "OUTSIDE_RESERVATION"
    assert gate[-1] is False


def test_body_with_its_braking_margin_fits_at_the_start_of_a_plan():
    """Found by the two-robot harness: at each replan the robot stands at the first sample, its rear inside the grant."""
    geom = stc.SemanticGeometry.from_payload(LANE_GEOMETRY)
    for offset in (0.0, 0.03, 0.07, 0.11):
        path = [(offset + x / 10.0, 0.0, 0.0) for x in range(0, 31)]
        client, _, _, clock = make()
        client.zones = []
        client.on_plan(path, goal=path[-1], command_id=CMD)
        assert client.on_authority_payload(
            grant_for(client, clock, geometry=LANE_GEOMETRY, resources=sorted(stc.collect_resources(path, [], geom))),
            target_command_id=CMD,
        )
        assert client.tick((offset, 0.0, 0.0)) is True, offset


def test_turning_across_the_lane_rectangle_closes_the_gate():
    client, _ = _lane_grant()
    assert client.tick((1.5, 0.08, 0.0)) is True
    assert client.tick((1.5, 0.08, 1.57)) is False


def test_a_spin_in_the_middle_of_a_lane_is_refused_but_allowed_at_a_disc():
    client, _ = _lane_grant()
    assert client.recovery_block_reason((1.5, 0.08, 0.0), "spin") == "SPIN_OUTSIDE_RESERVATION"
    assert client.recovery_block_reason((1.5, 0.08, 0.0), "backup", 0.10) is None
    assert client.recovery_block_reason((0.0, 0.0, 0.0), "spin") is None


def test_a_cell_the_path_comes_back_to_is_not_released():
    """Seen on the rig, turning at SHELF-B-01: the robot left a cell, its path re-entered it, the gate closed for 20 s."""
    out = [(x / 10.0, 0.0, 0.0) for x in range(0, 16)]
    back = [(1.5 - x / 10.0, 0.05, math.pi) for x in range(1, 16)]
    client, _, gate, clock = make()
    client.authority_loss_timeout_sec = 1e6  # this test is about the cells, and the clock runs fast here
    client.on_plan(out + back, goal=back[-1], command_id=CMD)
    assert client.on_authority_payload(grant_for(client, clock), target_command_id=CMD)
    start_cells = stc.footprint_resources(0.0, 0.0, GEOMETRY["footprintRadiusMeters"], ZONES, GEOMETRY["cellSizeMeters"])
    for x in [i / 10.0 for i in range(0, 16)]:
        client.resources_cleared_at(x, 0.0)
        clock.advance(client.clear_confirm_sec + 0.05)
        client.resources_cleared_at(x, 0.0)
    assert not (start_cells & client.released_resources)
    for x in [1.5 - i / 10.0 for i in range(1, 16)]:
        assert client.tick((x, 0.05, math.pi)) is True, x
        client.resources_cleared_at(x, 0.05)


def test_pose_noise_does_not_prematurely_clear_resources():
    """One sample outside a cell then back inside before clear_confirm_sec must not clear it."""
    client, _, _, clock = granted()
    client.resources_cleared_at(0.0, 0.0)
    x = 0.0
    while x < 3.0:
        client.resources_cleared_at(x, 0.0)
        if client._pending_clear:
            pending = set(client._pending_clear)
            x_back = max(0.0, x - 0.08)
            assert client.resources_cleared_at(x_back, 0.0) == []
            # Noise cancelled the pending set (footprint returned).
            assert not (pending & set(client._pending_clear))
            clock.advance(client.clear_confirm_sec + 0.1)
            assert client.resources_cleared_at(x_back, 0.0) == []
            assert not (pending & client.released_resources)
            return
        x += 0.02
    raise AssertionError("never pending")


def test_a_newer_grant_listing_a_resource_makes_it_usable_again():
    client, _, _, clock = granted()
    x, cleared = _drive_until_something_is_released(client, clock)
    assert client.released_resources == set(cleared)
    assert client.on_authority_payload(grant_for(client, clock, authority="auth-2"), target_command_id=CMD)
    assert client.released_resources == set()
    assert client.tick(_at_x(x - 0.15)) is True


def test_a_newer_grant_without_the_resource_keeps_it_released():
    client, _, _, clock = granted()
    x, cleared = _drive_until_something_is_released(client, clock)
    remaining = sorted(set(client.grant.resource_ids) - set(cleared))
    assert client.on_authority_payload(grant_for(client, clock, authority="auth-2", resources=remaining), target_command_id=CMD)
    assert client.released_resources == set(cleared)


def test_finishing_the_command_forgets_released_resources():
    client, _, _, clock = granted()
    _drive_until_something_is_released(client, clock)
    client.on_command_finished(CMD)
    assert client.released_resources == set()
