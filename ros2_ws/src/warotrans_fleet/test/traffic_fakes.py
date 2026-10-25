"""Builder/fake dùng chung cho test Active Mode (không cần ROS)."""

from __future__ import annotations

from typing import Any, Mapping, Optional, Sequence

from warotrans_fleet.authority_store import Authority, AuthorityStore, compute_path_hash
from warotrans_fleet.command_contract import NAVIGATE_TO_POSE, CommandEnvelope
from warotrans_fleet.route_enforcer import FakeRouteGraphPort
from warotrans_fleet.segment_executor import FakeSegmentExecutor
from warotrans_fleet.traffic_client import TrafficClient

ROBOT = "RBT-001"
GRAPH_HASH = "graphhash-abc"
MAP_CODE = "MAP-1"
ALL_EDGES = (1, 2, 3, 4, 5)


def make_authority(
    *,
    edges: Sequence[int] = (1, 2),
    epoch: int = 1,
    seq: int = 1,
    expires_at: Optional[float] = None,
    authority_id: str = "auth-1",
    command_id: Optional[str] = None,
    graph_hash: str = GRAPH_HASH,
    map_code: Optional[str] = MAP_CODE,
) -> Authority:
    return Authority(
        authority_id=authority_id,
        epoch=epoch,
        seq=seq,
        graph_hash=graph_hash,
        path_hash=compute_path_hash(graph_hash, edges),
        segment_edge_ids=tuple(edges),
        map_version_code=map_code,
        expires_at=expires_at,
        command_id=command_id,
    )


def authority_payload(
    *,
    edges: Sequence[int] = (1, 2),
    epoch: int = 1,
    seq: int = 1,
    expires_at: Any = None,
    authority_id: str = "auth-1",
    graph_hash: str = GRAPH_HASH,
    path_hash: Optional[str] = None,
) -> dict[str, Any]:
    body: dict[str, Any] = {
        "authorityId": authority_id,
        "epoch": epoch,
        "seq": seq,
        "graphHash": graph_hash,
        "pathHash": path_hash if path_hash is not None else compute_path_hash(graph_hash, edges),
        "segmentEdgeIds": list(edges),
        "mapVersionCode": MAP_CODE,
        "routeId": "route-1",
        "routeVersion": 2,
        "goalSafeStopNodeId": 7,
        "finalGoal": {"x": 1.0, "y": 2.0, "yaw": 0.5},
        "speedLimits": {"linear": 0.1},
    }
    if expires_at is not None:
        body["expiresAt"] = expires_at
    return {"authority": body}


def make_envelope(
    *,
    command_id: str = "cmd-1",
    message_id: str = "msg-1",
    command_type: str = NAVIGATE_TO_POSE,
    payload: Optional[Mapping[str, Any]] = None,
) -> CommandEnvelope:
    return CommandEnvelope(
        schema_version=1,
        message_id=message_id,
        command_id=command_id,
        robot_code=ROBOT,
        command_type=command_type,
        issued_at=None,
        job_assignment_id=None,
        job_step_id=None,
        payload=dict(payload or {}),
    )


class Clock:
    """Đồng hồ giả cho cả wall và monotonic."""

    def __init__(self, now: float = 1000.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


class Harness:
    def __init__(self, *, route_ok: bool = True, with_pose: bool = True, segment_goal: Any = None) -> None:
        self.clock = Clock()
        self.store = AuthorityStore(persist=False, clock=self.clock)
        self.route_port = FakeRouteGraphPort(adjust_ok=route_ok)
        self.executor = FakeSegmentExecutor()
        self.events: list[dict[str, Any]] = []
        self.results: list[tuple[str, str, Optional[str]]] = []
        self.pose: Optional[tuple[float, float, float]] = (0.0, 0.0, 0.0) if with_pose else None
        self.client = TrafficClient(
            store=self.store,
            route_port=self.route_port,
            executor=self.executor,
            progress_publisher=self.events.append,
            pose_provider=lambda: self.pose,
            all_edge_ids=ALL_EDGES,
            clock=self.clock,
            monotonic=self.clock,
            result_callback=lambda c, o, e: self.results.append((c, o, e)),
            segment_goal_resolver=segment_goal,
        )

    def event_names(self) -> list[str]:
        return [e["event"] for e in self.events]

    def start(self, authority: Optional[Authority] = None, *, command_id: str = "cmd-1", message_id: str = "msg-1"):
        authority = authority or make_authority()
        return self.client.start_navigate(
            make_envelope(command_id=command_id, message_id=message_id),
            authority,
            MAP_CODE,
            GRAPH_HASH,
        )
