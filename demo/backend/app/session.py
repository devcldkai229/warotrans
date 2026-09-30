"""In-RAM semantic map session (Phase 1 + Phase 2). No database."""

from __future__ import annotations

import uuid
from enum import Enum
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field, field_validator


class EndpointType(str, Enum):
    PICKUP = "PICKUP"
    DROPOFF = "DROPOFF"
    PARK = "PARK"
    QUEUE = "QUEUE"
    WAIT = "WAIT"


class LaneDirection(str, Enum):
    ONE_WAY = "ONE_WAY"
    BIDIRECTIONAL = "BIDIRECTIONAL"


class ZoneType(str, Enum):
    JUNCTION = "JUNCTION"
    KEEP_OUT = "KEEP_OUT"
    SPEED_LIMIT = "SPEED_LIMIT"
    SINGLE_ROBOT = "SINGLE_ROBOT"


class NavStatus(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


# --- Endpoint (Phase 1) ---


class EndpointCreate(BaseModel):
    name: str = Field(min_length=1, max_length=64)
    type: EndpointType
    x: float
    y: float
    yaw: float = 0.0
    enabled: bool = True


class EndpointUpdate(BaseModel):
    name: Optional[str] = Field(default=None, min_length=1, max_length=64)
    type: Optional[EndpointType] = None
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None
    enabled: Optional[bool] = None


class Endpoint(BaseModel):
    id: str
    name: str
    type: EndpointType
    x: float
    y: float
    yaw: float
    enabled: bool = True


# --- Lane (Phase 2) ---


class LaneCreate(BaseModel):
    name: str = Field(min_length=1, max_length=64)
    startX: float
    startY: float
    endX: float
    endY: float
    widthMeters: float = Field(default=0.6, gt=0.05, le=5.0)
    direction: LaneDirection = LaneDirection.BIDIRECTIONAL
    enabled: bool = True
    speedLimit: Optional[float] = Field(default=None, gt=0.0)


class LaneUpdate(BaseModel):
    name: Optional[str] = Field(default=None, min_length=1, max_length=64)
    startX: Optional[float] = None
    startY: Optional[float] = None
    endX: Optional[float] = None
    endY: Optional[float] = None
    widthMeters: Optional[float] = Field(default=None, gt=0.05, le=5.0)
    direction: Optional[LaneDirection] = None
    enabled: Optional[bool] = None
    speedLimit: Optional[float] = Field(default=None, gt=0.0)


class Lane(BaseModel):
    id: str
    name: str
    startX: float
    startY: float
    endX: float
    endY: float
    widthMeters: float
    direction: LaneDirection
    enabled: bool = True
    speedLimit: Optional[float] = None


# --- TrafficZone (Phase 2) ---


class ZonePoint(BaseModel):
    x: float
    y: float


class ZoneCreate(BaseModel):
    name: str = Field(min_length=1, max_length=64)
    type: ZoneType
    polygon: List[ZonePoint]
    enabled: bool = True
    maxSpeedMps: Optional[float] = Field(default=None, gt=0.0)
    capacity: Optional[int] = Field(default=None, ge=1)

    @field_validator("polygon")
    @classmethod
    def _min_vertices(cls, v: List[ZonePoint]) -> List[ZonePoint]:
        if len(v) < 3:
            raise ValueError("polygon needs at least 3 vertices")
        return v


class ZoneUpdate(BaseModel):
    name: Optional[str] = Field(default=None, min_length=1, max_length=64)
    type: Optional[ZoneType] = None
    polygon: Optional[List[ZonePoint]] = None
    enabled: Optional[bool] = None
    maxSpeedMps: Optional[float] = Field(default=None, gt=0.0)
    capacity: Optional[int] = Field(default=None, ge=1)


class TrafficZone(BaseModel):
    id: str
    name: str
    type: ZoneType
    polygon: List[ZonePoint]
    enabled: bool = True
    maxSpeedMps: Optional[float] = None
    capacity: Optional[int] = None


# --- Nav / robot (Phase 1) ---


class NavState(BaseModel):
    status: NavStatus = NavStatus.IDLE
    endpoint_id: Optional[str] = None
    message: str = ""
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    goal_yaw: Optional[float] = None
    mode: str = "idle"  # idle | endpoint | lanes
    lane_ids: List[str] = Field(default_factory=list)
    speed_limit_mps: Optional[float] = None
    speed_limit_percent: Optional[float] = None
    filters_applied_at: Optional[float] = None


class LaneNavigateRequest(BaseModel):
    lane_ids: List[str] = Field(min_length=1)
    spacing_m: float = Field(default=0.35, gt=0.05, le=2.0)


class RobotPose(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    timestamp: float = 0.0
    valid: bool = False


class PathPose(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class NavPath(BaseModel):
    frame_id: str = "map"
    poses: List[PathPose] = Field(default_factory=list)


class MapVersion(BaseModel):
    """Semantic facility map snapshot — ROS map metres only."""

    version: int = 2
    frame_id: str = "map"
    endpoints: List[Endpoint] = Field(default_factory=list)
    lanes: List[Lane] = Field(default_factory=list)
    zones: List[TrafficZone] = Field(default_factory=list)


class SessionStore:
    """Process-local semantic map. Restart clears all data."""

    def __init__(self) -> None:
        self._endpoints: Dict[str, Endpoint] = {}
        self._lanes: Dict[str, Lane] = {}
        self._zones: Dict[str, TrafficZone] = {}
        self.nav = NavState()
        self.robot = RobotPose()
        self.path = NavPath()

    # --- endpoints ---
    def list_endpoints(self) -> List[Endpoint]:
        return list(self._endpoints.values())

    def get(self, endpoint_id: str) -> Optional[Endpoint]:
        return self._endpoints.get(endpoint_id)

    def create(self, body: EndpointCreate) -> Endpoint:
        ep = Endpoint(
            id=str(uuid.uuid4()),
            name=body.name,
            type=body.type,
            x=body.x,
            y=body.y,
            yaw=body.yaw,
            enabled=body.enabled,
        )
        self._endpoints[ep.id] = ep
        return ep

    def update(self, endpoint_id: str, body: EndpointUpdate) -> Optional[Endpoint]:
        ep = self._endpoints.get(endpoint_id)
        if ep is None:
            return None
        data = ep.model_dump()
        data.update(body.model_dump(exclude_unset=True))
        updated = Endpoint(**data)
        self._endpoints[endpoint_id] = updated
        return updated

    def delete(self, endpoint_id: str) -> bool:
        return self._endpoints.pop(endpoint_id, None) is not None

    # --- lanes ---
    def list_lanes(self) -> List[Lane]:
        return list(self._lanes.values())

    def get_lane(self, lane_id: str) -> Optional[Lane]:
        return self._lanes.get(lane_id)

    def create_lane(self, body: LaneCreate) -> Lane:
        lane = Lane(id=str(uuid.uuid4()), **body.model_dump())
        self._lanes[lane.id] = lane
        return lane

    def update_lane(self, lane_id: str, body: LaneUpdate) -> Optional[Lane]:
        lane = self._lanes.get(lane_id)
        if lane is None:
            return None
        data = lane.model_dump()
        data.update(body.model_dump(exclude_unset=True))
        updated = Lane(**data)
        self._lanes[lane_id] = updated
        return updated

    def delete_lane(self, lane_id: str) -> bool:
        return self._lanes.pop(lane_id, None) is not None

    # --- zones ---
    def list_zones(self) -> List[TrafficZone]:
        return list(self._zones.values())

    def get_zone(self, zone_id: str) -> Optional[TrafficZone]:
        return self._zones.get(zone_id)

    def create_zone(self, body: ZoneCreate) -> TrafficZone:
        data = body.model_dump()
        if body.type == ZoneType.SINGLE_ROBOT and data.get("capacity") is None:
            data["capacity"] = 1
        zone = TrafficZone(id=str(uuid.uuid4()), **data)
        self._zones[zone.id] = zone
        return zone

    def update_zone(self, zone_id: str, body: ZoneUpdate) -> Optional[TrafficZone]:
        zone = self._zones.get(zone_id)
        if zone is None:
            return None
        data = zone.model_dump()
        patch = body.model_dump(exclude_unset=True)
        if "polygon" in patch and patch["polygon"] is not None:
            if len(patch["polygon"]) < 3:
                raise ValueError("polygon needs at least 3 vertices")
        data.update(patch)
        updated = TrafficZone(**data)
        self._zones[zone_id] = updated
        return updated

    def delete_zone(self, zone_id: str) -> bool:
        return self._zones.pop(zone_id, None) is not None

    # --- export / import ---
    def export_map(self) -> MapVersion:
        return MapVersion(
            version=2,
            frame_id="map",
            endpoints=self.list_endpoints(),
            lanes=self.list_lanes(),
            zones=self.list_zones(),
        )

    def import_map(self, payload: MapVersion | Dict[str, Any]) -> MapVersion:
        if isinstance(payload, dict):
            payload = MapVersion.model_validate(payload)
        self._endpoints = {e.id: e for e in payload.endpoints}
        self._lanes = {ln.id: ln for ln in payload.lanes}
        self._zones = {z.id: z for z in payload.zones}
        return self.export_map()


store = SessionStore()
