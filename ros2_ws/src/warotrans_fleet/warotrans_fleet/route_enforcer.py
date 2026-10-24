"""Route-grant enforcement before / during NAVIGATE_TO_POSE.

Real ROS clients (DynamicEdges + ComputeRoute) are injected; unit tests use fakes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Sequence

from warotrans_fleet.command_contract import CommandRejected
from warotrans_fleet.route_grant import (
    REASON_ROUTE_GRAPH_MISMATCH,
    REASON_ROUTE_NOT_AUTHORIZED,
    REASON_ROUTE_REQUIRED,
    GraphEdge,
    RouteGrant,
    edges_to_close,
    load_graph_edges,
    load_graph_hash,
    outside_corridor,
    parse_route_grant,
    route_matches_grant,
)


class RouteGraphPort(Protocol):
    """Nav2 route_server DynamicEdges + ComputeRoute surface used by the gateway."""

    def adjust_edges(self, *, close_ids: Sequence[int], open_ids: Sequence[int]) -> bool: ...

    def compute_route_edge_ids(
        self,
        *,
        frame_id: str,
        start_x: float,
        start_y: float,
        start_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
    ) -> list[int]: ...


@dataclass
class RouteEnforcerConfig:
    enabled: bool = False
    graph_path: str = ""
    graph_hash: str = ""
    corridor_buffer_m: float = 0.35
    grant_loss_timeout_sec: float = 3.0


class RouteEnforcer:
    def __init__(
        self,
        *,
        config: RouteEnforcerConfig,
        graph_port: Optional[RouteGraphPort] = None,
        pose_provider: Optional[Callable[[], Optional[tuple[float, float]]]] = None,
        mqtt_connected: Optional[Callable[[], bool]] = None,
        log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._config = config
        self._port = graph_port
        self._pose_provider = pose_provider or (lambda: None)
        self._mqtt_connected = mqtt_connected or (lambda: True)
        self._log = log or (lambda _m: None)
        self._edges: list[GraphEdge] = []
        self._all_ids: list[int] = []
        self._expected_hash: Optional[str] = None
        self._active_grant: Optional[RouteGrant] = None
        self._disconnect_since: Optional[float] = None

        if self.enabled:
            self._reload_graph()

    @property
    def enabled(self) -> bool:
        if self._config.enabled:
            return True
        return bool(self._config.graph_path and self._config.graph_path.strip())

    @property
    def active_grant(self) -> Optional[RouteGrant]:
        return self._active_grant

    def _reload_graph(self) -> None:
        path = self._config.graph_path.strip()
        if not path:
            self._edges = []
            self._all_ids = []
            self._expected_hash = self._config.graph_hash.strip().lower() or None
            return
        self._edges = load_graph_edges(path)
        self._all_ids = sorted({e.edge_id for e in self._edges})
        self._expected_hash = load_graph_hash(
            explicit=self._config.graph_hash or None,
            graph_path=path,
        )

    def prepare_navigate(self, payload: dict, *, goal_x: float, goal_y: float, goal_yaw: float, frame_id: str) -> RouteGrant:
        """Validate grant, close non-granted edges, verify ComputeRoute matches. Raises CommandRejected."""
        if not self.enabled:
            raise RuntimeError("prepare_navigate called while route enforcement is off")

        grant = parse_route_grant(payload)
        if grant is None:
            raise CommandRejected(REASON_ROUTE_REQUIRED, "route enforcement requires payload.route")

        if self._expected_hash and grant.graph_hash.lower() != self._expected_hash:
            raise CommandRejected(
                REASON_ROUTE_GRAPH_MISMATCH,
                f"route.graphHash {grant.graph_hash!r} != loaded {self._expected_hash!r}",
            )

        if self._port is None:
            raise CommandRejected(REASON_ROUTE_NOT_AUTHORIZED, "route graph port unavailable")

        close_ids = edges_to_close(self._all_ids, grant.edge_ids)
        open_ids = list(grant.edge_ids)
        if not self._port.adjust_edges(close_ids=close_ids, open_ids=open_ids):
            raise CommandRejected(REASON_ROUTE_NOT_AUTHORIZED, "DynamicEdges adjust_edges failed")

        pose = self._pose_provider()
        start_x, start_y, start_yaw = (pose[0], pose[1], 0.0) if pose is not None else (goal_x, goal_y, goal_yaw)
        computed = self._port.compute_route_edge_ids(
            frame_id=frame_id,
            start_x=start_x,
            start_y=start_y,
            start_yaw=start_yaw,
            goal_x=goal_x,
            goal_y=goal_y,
            goal_yaw=goal_yaw,
        )
        if not route_matches_grant(computed, grant):
            raise CommandRejected(
                REASON_ROUTE_NOT_AUTHORIZED,
                f"ComputeRoute edges {list(computed)} != grant {list(grant.edge_ids)}",
            )

        self._active_grant = grant
        self._disconnect_since = None
        self._log(f"[RouteGrantApplied] edges={list(grant.edge_ids)} hash={grant.graph_hash[:12]}…")
        return grant

    def clear_active(self) -> None:
        self._active_grant = None
        self._disconnect_since = None

    def monitor_tick(self, now: float) -> Optional[str]:
        """Return error code when active grant should be canceled, else None."""
        grant = self._active_grant
        if grant is None:
            return None

        if not self._mqtt_connected():
            if self._disconnect_since is None:
                self._disconnect_since = now
            elif now - self._disconnect_since >= self._config.grant_loss_timeout_sec:
                from warotrans_fleet.route_grant import ERROR_GRANT_LOSS

                return ERROR_GRANT_LOSS
        else:
            self._disconnect_since = None

        pose = self._pose_provider()
        if pose is not None and outside_corridor(
            pose[0],
            pose[1],
            self._edges,
            grant.edge_ids,
            buffer_m=self._config.corridor_buffer_m,
        ):
            from warotrans_fleet.route_grant import ERROR_CORRIDOR_VIOLATION

            return ERROR_CORRIDOR_VIOLATION
        return None


class FakeRouteGraphPort:
    """Test double for DynamicEdges + ComputeRoute."""

    def __init__(self, *, computed_edges: Optional[Sequence[int]] = None, adjust_ok: bool = True) -> None:
        self.computed_edges = list(computed_edges or [])
        self.adjust_ok = adjust_ok
        self.last_close: list[int] = []
        self.last_open: list[int] = []
        self.compute_calls = 0

    def adjust_edges(self, *, close_ids: Sequence[int], open_ids: Sequence[int]) -> bool:
        self.last_close = list(close_ids)
        self.last_open = list(open_ids)
        return self.adjust_ok

    def compute_route_edge_ids(
        self,
        *,
        frame_id: str,
        start_x: float,
        start_y: float,
        start_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
    ) -> list[int]:
        self.compute_calls += 1
        return list(self.computed_edges)
