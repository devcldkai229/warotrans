"""Reverse densify + maneuver parse (CompilerVersion 10 contract)."""

import math

import pytest
from warotrans_fleet.authority_store import ManeuverStep, parse_authority, parse_maneuvers
from warotrans_fleet.command_contract import CommandRejected
from warotrans_fleet.nav2_segment_executor import densify_path, densify_path_reversed
from warotrans_fleet.route_grant import compute_path_hash


def test_densify_path_reversed_flips_travel_direction():
    line = [[(0.0, 0.0), (1.0, 0.0)]]
    fwd = densify_path(line)
    rev = densify_path_reversed(line)
    assert fwd[0][:2] == (0.0, 0.0)
    assert rev[0][:2] == (1.0, 0.0)
    assert math.isclose(fwd[0][2], 0.0, abs_tol=1e-9)
    assert abs(abs(rev[0][2]) - math.pi) < 1e-6


def test_parse_maneuvers_empty_is_legacy():
    assert parse_maneuvers(None) == ()
    assert parse_maneuvers([]) == ()


def test_parse_maneuvers_forward_reverse_spin():
    steps = parse_maneuvers(
        [
            {"kind": "Forward", "edgeIds": [1, 2]},
            {"kind": "Spin", "edgeIds": [], "yawTarget": 1.57, "resourceIds": ["node:3"]},
            {"kind": "Reverse", "edgeIds": [4]},
        ]
    )
    assert [s.kind for s in steps] == ["Forward", "Spin", "Reverse"]
    assert steps[1].yaw_target == pytest.approx(1.57)
    assert steps[2].edge_ids == (4,)


def test_parse_maneuvers_rejects_unknown_kind():
    with pytest.raises(CommandRejected):
        parse_maneuvers([{"kind": "ContinuousArc", "edgeIds": [1]}])


def test_authority_resolved_maneuvers_legacy_forward():
    gh = "abc"
    edges = (10, 11)
    ph = compute_path_hash(gh, edges)
    a = parse_authority(
        {
            "authority": {
                "authorityId": "a1",
                "epoch": 1,
                "seq": 1,
                "graphHash": gh,
                "pathHash": ph,
                "segmentEdgeIds": list(edges),
            }
        }
    )
    assert a.maneuvers == ()
    resolved = a.resolved_maneuvers()
    assert resolved == (ManeuverStep(kind="Forward", edge_ids=edges),)
