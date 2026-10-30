"""Route-grant helpers for NAVIGATE_TO_POSE (pure Python — no ROS).

When route enforcement is on, the fleet payload carries ``route: { graphHash, edgeIds }``.
The robot closes every other graph edge, recomputes a route, and rejects if it differs.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

from warotrans_fleet.command_contract import CommandRejected, REASON_INVALID_PAYLOAD

REASON_ROUTE_REQUIRED = "ROUTE_REQUIRED"
REASON_ROUTE_NOT_AUTHORIZED = "ROUTE_NOT_AUTHORIZED"
REASON_ROUTE_GRAPH_MISMATCH = "ROUTE_GRAPH_MISMATCH"
ERROR_CORRIDOR_VIOLATION = "CORRIDOR_VIOLATION"
ERROR_GRANT_LOSS = "GRANT_LOSS"


def compute_path_hash(graph_hash: str, edge_ids: Iterable[int]) -> str:
    """sha256 hex (lowercase) của ``graphHash|id,id,...`` — dùng chung cho RouteGrant và Authority.

    Backend phải dùng cùng công thức; graph_hash được giữ nguyên (không đổi hoa/thường).
    """
    text = f"{graph_hash}|{','.join(str(int(e)) for e in edge_ids)}"
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class RouteGrant:
    graph_hash: str
    edge_ids: tuple[int, ...]

    @property
    def path_hash(self) -> str:
        return compute_path_hash(self.graph_hash, self.edge_ids)


@dataclass(frozen=True)
class GraphEdge:
    edge_id: int
    x0: float
    y0: float
    x1: float
    y1: float
    #: Full LineString vertices when available (FollowPath); endpoints alone if empty.
    points: tuple[tuple[float, float], ...] = ()


def parse_route_grant(payload: Mapping[str, Any]) -> Optional[RouteGrant]:
    """Parse optional ``payload.route``. Returns None when the key is absent."""
    if "route" not in payload or payload.get("route") is None:
        return None
    route = payload["route"]
    if not isinstance(route, Mapping):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.route' must be an object")

    graph_hash = route.get("graphHash")
    if not isinstance(graph_hash, str) or not graph_hash.strip():
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.route.graphHash' is required")

    raw_ids = route.get("edgeIds")
    if not isinstance(raw_ids, list):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.route.edgeIds' must be an array")

    edge_ids: list[int] = []
    for item in raw_ids:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or int(item) != item:
            raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.route.edgeIds' must be integers")
        edge_ids.append(int(item))

    return RouteGrant(graph_hash=graph_hash.strip(), edge_ids=tuple(edge_ids))


def read_embedded_graph_hash(graph_path: str) -> Optional[str]:
    """``properties.graphHash`` written by the backend route-graph export (FeatureCollection level)."""
    try:
        data = json.loads(Path(graph_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    props = data.get("properties") if isinstance(data, dict) else None
    value = props.get("graphHash") if isinstance(props, Mapping) else None
    return value.strip().lower() if isinstance(value, str) and value.strip() else None


def load_graph_hash(*, explicit: Optional[str], graph_path: Optional[str]) -> Optional[str]:
    """Expected graph hash: param, else the hash the backend embedded in the GeoJSON, else a sidecar file.

    Never a hash of the file bytes: the backend hashes its canonical compile input, so a byte hash can never
    match and would reject every grant (audit G3). ``None`` means "unknown" — Active Mode then refuses grants.
    """
    if explicit and explicit.strip():
        return explicit.strip().lower()
    if not graph_path:
        return None
    embedded = read_embedded_graph_hash(graph_path)
    if embedded:
        return embedded
    path = Path(graph_path)
    for suffix in (".sha256", ".hash"):
        # Prefer route_graph.geojson.sha256 then route_graph.sha256
        candidates = [
            Path(str(path) + suffix),
            path.with_suffix(suffix),
        ]
        for candidate in candidates:
            if candidate.is_file():
                text = candidate.read_text(encoding="utf-8").strip().split()[0]
                return text.lower()
    return None


def load_graph_edges(graph_path: str) -> list[GraphEdge]:
    """Load edge id + segment endpoints from a nav2_route GeoJSON graph file."""
    data = json.loads(Path(graph_path).read_text(encoding="utf-8"))
    features = data.get("features") if isinstance(data, dict) else None
    if not isinstance(features, list):
        raise ValueError(f"route graph {graph_path!r} has no features[]")

    edges: list[GraphEdge] = []
    for feature in features:
        if not isinstance(feature, Mapping):
            continue
        props = feature.get("properties") or {}
        geom = feature.get("geometry") or {}
        if not isinstance(props, Mapping) or not isinstance(geom, Mapping):
            continue
        if geom.get("type") not in ("LineString", "MultiLineString"):
            continue
        edge_id = props.get("id")
        if not isinstance(edge_id, (int, float)) or isinstance(edge_id, bool):
            continue
        coords = _first_line_coords(geom)
        if coords is None or len(coords) < 2:
            continue
        (x0, y0), (x1, y1) = coords[0], coords[-1]
        pts = tuple((float(x), float(y)) for x, y in coords)
        edges.append(GraphEdge(int(edge_id), float(x0), float(y0), float(x1), float(y1), pts))
    return edges


def _first_line_coords(geom: Mapping[str, Any]) -> Optional[list[tuple[float, float]]]:
    coords = geom.get("coordinates")
    gtype = geom.get("type")
    if gtype == "LineString" and isinstance(coords, list) and coords:
        return [(float(p[0]), float(p[1])) for p in coords if isinstance(p, (list, tuple)) and len(p) >= 2]
    if gtype == "MultiLineString" and isinstance(coords, list) and coords:
        first = coords[0]
        if isinstance(first, list) and first:
            return [(float(p[0]), float(p[1])) for p in first if isinstance(p, (list, tuple)) and len(p) >= 2]
    return None


def edges_to_close(all_edge_ids: Iterable[int], granted: Sequence[int]) -> list[int]:
    granted_set = set(granted)
    return sorted(eid for eid in all_edge_ids if eid not in granted_set)


def route_matches_grant(computed_edge_ids: Sequence[int], grant: RouteGrant) -> bool:
    return tuple(computed_edge_ids) == grant.edge_ids


def point_to_segment_distance(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
    dx = x1 - x0
    dy = y1 - y0
    length2 = dx * dx + dy * dy
    if length2 < 1e-18:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / length2))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def distance_to_corridor(px: float, py: float, edges: Sequence[GraphEdge], granted_ids: Sequence[int]) -> float:
    """Minimum distance from pose to any granted edge segment. inf if no granted edges."""
    granted = set(granted_ids)
    best = float("inf")
    for edge in edges:
        if edge.edge_id not in granted:
            continue
        best = min(best, point_to_segment_distance(px, py, edge.x0, edge.y0, edge.x1, edge.y1))
    return best


def outside_corridor(
    px: float,
    py: float,
    edges: Sequence[GraphEdge],
    granted_ids: Sequence[int],
    *,
    buffer_m: float,
) -> bool:
    if not granted_ids:
        return False
    return distance_to_corridor(px, py, edges, granted_ids) > buffer_m
