"""Route-grant parse / corridor / enforcer — no ROS, no real robot."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from fleet_fakes import Harness, navigate

from warotrans_fleet.command_contract import CommandRejected
from warotrans_fleet.route_enforcer import FakeRouteGraphPort, RouteEnforcer, RouteEnforcerConfig
from warotrans_fleet.route_grant import (
    ERROR_CORRIDOR_VIOLATION,
    ERROR_GRANT_LOSS,
    REASON_ROUTE_GRAPH_MISMATCH,
    REASON_ROUTE_NOT_AUTHORIZED,
    REASON_ROUTE_REQUIRED,
    distance_to_corridor,
    edges_to_close,
    load_graph_edges,
    outside_corridor,
    parse_route_grant,
    point_to_segment_distance,
)


def test_parse_route_grant_ok():
    grant = parse_route_grant({"route": {"graphHash": "abc", "edgeIds": [1, 2, 3]}})
    assert grant is not None
    assert grant.graph_hash == "abc"
    assert grant.edge_ids == (1, 2, 3)


def test_parse_route_grant_absent():
    assert parse_route_grant({"x": 1.0}) is None


def test_parse_route_grant_rejects_bad_ids():
    with pytest.raises(CommandRejected):
        parse_route_grant({"route": {"graphHash": "h", "edgeIds": ["a"]}})


def test_edges_to_close_and_corridor_distance(tmp_path: Path):
    geo = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"id": 10},
                "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [2.0, 0.0]]},
            },
            {
                "type": "Feature",
                "properties": {"id": 11},
                "geometry": {"type": "MultiLineString", "coordinates": [[[2.0, 0.0], [2.0, 2.0]]]},
            },
        ],
    }
    path = tmp_path / "graph.geojson"
    path.write_text(json.dumps(geo), encoding="utf-8")
    edges = load_graph_edges(str(path))
    assert {e.edge_id for e in edges} == {10, 11}
    assert edges_to_close([10, 11], [10]) == [11]
    assert point_to_segment_distance(1.0, 0.1, 0.0, 0.0, 2.0, 0.0) == pytest.approx(0.1)
    assert distance_to_corridor(1.0, 0.0, edges, [10]) == pytest.approx(0.0)
    assert outside_corridor(1.0, 1.0, edges, [10], buffer_m=0.3)


def test_enforcer_requires_route_when_enabled():
    port = FakeRouteGraphPort(computed_edges=[1])
    enforcer = RouteEnforcer(
        config=RouteEnforcerConfig(enabled=True, graph_hash="deadbeef"),
        graph_port=port,
    )
    with pytest.raises(CommandRejected) as exc:
        enforcer.prepare_navigate({}, goal_x=1, goal_y=0, goal_yaw=0, frame_id="map")
    assert exc.value.reason_code == REASON_ROUTE_REQUIRED


def test_enforcer_rejects_hash_mismatch():
    port = FakeRouteGraphPort(computed_edges=[1])
    enforcer = RouteEnforcer(
        config=RouteEnforcerConfig(enabled=True, graph_hash="expected"),
        graph_port=port,
    )
    with pytest.raises(CommandRejected) as exc:
        enforcer.prepare_navigate(
            {"route": {"graphHash": "other", "edgeIds": [1]}},
            goal_x=1,
            goal_y=0,
            goal_yaw=0,
            frame_id="map",
        )
    assert exc.value.reason_code == REASON_ROUTE_GRAPH_MISMATCH


def test_enforcer_rejects_route_not_matching_compute(tmp_path: Path):
    geo = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"id": 1},
                "geometry": {"type": "LineString", "coordinates": [[0, 0], [1, 0]]},
            },
            {
                "type": "Feature",
                "properties": {"id": 2},
                "geometry": {"type": "LineString", "coordinates": [[1, 0], [2, 0]]},
            },
        ],
    }
    path = tmp_path / "g.geojson"
    path.write_text(json.dumps(geo), encoding="utf-8")
    port = FakeRouteGraphPort(computed_edges=[1, 2])  # broader than grant
    enforcer = RouteEnforcer(
        config=RouteEnforcerConfig(enabled=True, graph_path=str(path), graph_hash="hh"),
        graph_port=port,
        pose_provider=lambda: (0.0, 0.0),
    )
    with pytest.raises(CommandRejected) as exc:
        enforcer.prepare_navigate(
            {"route": {"graphHash": "hh", "edgeIds": [1]}},
            goal_x=1,
            goal_y=0,
            goal_yaw=0,
            frame_id="map",
        )
    assert exc.value.reason_code == REASON_ROUTE_NOT_AUTHORIZED
    assert port.last_close == [2]
    assert port.last_open == [1]


def test_enforcer_applies_matching_grant(tmp_path: Path):
    geo = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"id": 7},
                "geometry": {"type": "LineString", "coordinates": [[0, 0], [2, 0]]},
            }
        ],
    }
    path = tmp_path / "g.geojson"
    path.write_text(json.dumps(geo), encoding="utf-8")
    port = FakeRouteGraphPort(computed_edges=[7])
    enforcer = RouteEnforcer(
        config=RouteEnforcerConfig(enabled=True, graph_path=str(path), graph_hash="ok"),
        graph_port=port,
        pose_provider=lambda: (0.0, 0.0),
    )
    grant = enforcer.prepare_navigate(
        {"route": {"graphHash": "ok", "edgeIds": [7]}},
        goal_x=2,
        goal_y=0,
        goal_yaw=0,
        frame_id="map",
    )
    assert grant.edge_ids == (7,)
    assert port.compute_calls == 1


def test_monitor_corridor_and_grant_loss(tmp_path: Path):
    geo = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"id": 1},
                "geometry": {"type": "LineString", "coordinates": [[0, 0], [2, 0]]},
            }
        ],
    }
    path = tmp_path / "g.geojson"
    path.write_text(json.dumps(geo), encoding="utf-8")
    port = FakeRouteGraphPort(computed_edges=[1])
    pose = {"xy": (1.0, 0.0)}
    mqtt_ok = {"v": True}
    enforcer = RouteEnforcer(
        config=RouteEnforcerConfig(
            enabled=True,
            graph_path=str(path),
            graph_hash="h",
            corridor_buffer_m=0.2,
            grant_loss_timeout_sec=1.0,
        ),
        graph_port=port,
        pose_provider=lambda: pose["xy"],
        mqtt_connected=lambda: mqtt_ok["v"],
    )
    enforcer.prepare_navigate(
        {"route": {"graphHash": "h", "edgeIds": [1]}},
        goal_x=2,
        goal_y=0,
        goal_yaw=0,
        frame_id="map",
    )
    assert enforcer.monitor_tick(0.0) is None
    pose["xy"] = (1.0, 1.0)
    assert enforcer.monitor_tick(0.1) == ERROR_CORRIDOR_VIOLATION

    pose["xy"] = (1.0, 0.0)
    mqtt_ok["v"] = False
    assert enforcer.monitor_tick(10.0) is None  # start disconnect clock
    assert enforcer.monitor_tick(11.1) == ERROR_GRANT_LOSS


def test_move_handler_rejects_without_route_when_enforcement_on(tmp_path: Path):
    path = tmp_path / "g.geojson"
    path.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "features": [
                    {
                        "type": "Feature",
                        "properties": {"id": 1},
                        "geometry": {"type": "LineString", "coordinates": [[0, 0], [1, 0]]},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    port = FakeRouteGraphPort(computed_edges=[1])
    enforcer = RouteEnforcer(
        config=RouteEnforcerConfig(enabled=True, graph_path=str(path), graph_hash="h"),
        graph_port=port,
        pose_provider=lambda: (0.0, 0.0),
    )
    h = Harness(route_enforcer=enforcer)
    h.send(navigate())
    assert h.recorder.acks[0]["accepted"] is False
    assert h.recorder.acks[0]["reasonCode"] == REASON_ROUTE_REQUIRED


def test_move_handler_accepts_with_matching_route(tmp_path: Path):
    path = tmp_path / "g.geojson"
    path.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "features": [
                    {
                        "type": "Feature",
                        "properties": {"id": 1},
                        "geometry": {"type": "LineString", "coordinates": [[0, 0], [4.25, 2.10]]},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    port = FakeRouteGraphPort(computed_edges=[1])
    enforcer = RouteEnforcer(
        config=RouteEnforcerConfig(enabled=True, graph_path=str(path), graph_hash="h"),
        graph_port=port,
        pose_provider=lambda: (0.0, 0.0),
    )
    h = Harness(route_enforcer=enforcer)
    h.send(navigate(route={"graphHash": "h", "edgeIds": [1]}))
    assert h.recorder.acks[0]["accepted"] is True
    assert len(h.nav.goals) == 1


def test_route_mode_off_unchanged():
    h = Harness()
    h.send(navigate())
    assert h.recorder.acks[0]["accepted"] is True
