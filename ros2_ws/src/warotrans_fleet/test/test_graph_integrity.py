"""Graph integrity (audit G1–G3): the robot only drives edges it has, contiguous, from the backend's graph."""

import json

from traffic_fakes import ALL_EDGES, GRAPH_HASH, MAP_CODE, Harness, make_authority, make_envelope
from warotrans_fleet import command_contract as cc
from warotrans_fleet.authority_store import compute_path_hash
from warotrans_fleet.route_grant import load_graph_hash
from warotrans_fleet.traffic_client import STATE_EXECUTING, STATE_IDLE, TrafficClient

# 1 → 2 → 3 → 5 form one line along y = 0; 4 lies elsewhere.
EDGE_ENDS = {
    1: ((0.0, 0.0), (1.0, 0.0)),
    2: ((1.0, 0.0), (2.0, 0.0)),
    3: ((2.0, 0.0), (3.0, 0.0)),
    4: ((5.0, 5.0), (6.0, 5.0)),
    5: ((3.0, 0.0), (4.0, 0.0)),
}


def _harness(*, require_hash: bool = True) -> Harness:
    h = Harness()
    h.client = TrafficClient(
        store=h.store,
        route_port=h.route_port,
        executor=h.executor,
        progress_publisher=h.events.append,
        pose_provider=lambda: h.pose,
        all_edge_ids=ALL_EDGES,
        clock=h.clock,
        monotonic=h.clock,
        result_callback=lambda c, o, e: h.results.append((c, o, e)),
        edge_endpoints=EDGE_ENDS,
        require_graph_hash=require_hash,
    )
    return h


def _start(h: Harness, edges, *, expected_hash=GRAPH_HASH):
    return h.client.start_navigate(make_envelope(), make_authority(edges=edges), MAP_CODE, expected_hash)


def _extend(h: Harness, edges, *, seq):
    return h.client.extend_authority(
        make_envelope(
            command_id="cmd-ext",
            message_id=f"ext-{seq}",
            command_type=cc.EXTEND_AUTHORITY,
            payload={
                "targetCommandId": "cmd-1",
                "authorityId": "auth-1",
                "epoch": 1,
                "seq": seq,
                "segmentEdgeIds": list(edges),
                "graphHash": GRAPH_HASH,
                "pathHash": compute_path_hash(GRAPH_HASH, edges),
            },
        ),
        expected_graph_hash=GRAPH_HASH,
    )


def test_contiguous_known_edges_are_accepted():
    h = _harness()
    assert _start(h, (1, 2)).accepted
    assert h.client.state == STATE_EXECUTING


def test_unknown_edge_is_refused_instead_of_bridged():
    h = _harness()
    decision = _start(h, (1, 99))
    assert not decision.accepted and decision.reason_code == cc.REASON_ROUTE_NOT_AUTHORIZED
    assert h.client.state == STATE_IDLE
    assert h.executor.follow_calls == []


def test_gap_between_edges_is_refused():
    h = _harness()
    decision = _start(h, (1, 4))
    assert not decision.accepted and decision.reason_code == cc.REASON_ROUTE_NOT_AUTHORIZED


def test_no_loaded_graph_hash_refuses_grants_when_required():
    h = _harness(require_hash=True)
    decision = _start(h, (1, 2), expected_hash=None)
    assert not decision.accepted and decision.reason_code == cc.REASON_ROUTE_GRAPH_MISMATCH


def test_extension_with_a_gap_is_refused():
    h = _harness()
    assert _start(h, (1, 2)).accepted
    decision = _extend(h, (1, 2, 4), seq=2)
    assert not decision.accepted and decision.reason_code == cc.REASON_ROUTE_NOT_AUTHORIZED


def test_contiguous_extension_is_accepted():
    h = _harness()
    assert _start(h, (1, 2)).accepted
    assert _extend(h, (1, 2, 3, 5), seq=2).accepted


def test_graph_hash_comes_from_the_exported_file(tmp_path):
    graph = tmp_path / "route_graph.geojson"
    graph.write_text(json.dumps({"type": "FeatureCollection", "properties": {"graphHash": "ABC123"}, "features": []}))
    assert load_graph_hash(explicit=None, graph_path=str(graph)) == "abc123"


def test_no_byte_hash_fallback_for_a_file_without_graph_hash(tmp_path):
    graph = tmp_path / "route_graph.geojson"
    graph.write_text(json.dumps({"type": "FeatureCollection", "features": []}))
    # A hash of the file bytes can never equal the backend's canonical hash (audit G3).
    assert load_graph_hash(explicit=None, graph_path=str(graph)) is None
