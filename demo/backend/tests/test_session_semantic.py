"""Phase 2: in-RAM semantic map CRUD + export/import."""

from app.session import (
    EndpointCreate,
    EndpointType,
    LaneCreate,
    LaneDirection,
    MapVersion,
    ZoneCreate,
    ZonePoint,
    ZoneType,
    SessionStore,
)


def test_lane_zone_export_import_roundtrip():
    store = SessionStore()
    ep = store.create(
        EndpointCreate(name="P1", type=EndpointType.PARK, x=1.0, y=2.0, yaw=0.5)
    )
    lane = store.create_lane(
        LaneCreate(
            name="Main",
            startX=0.0,
            startY=0.0,
            endX=2.0,
            endY=0.0,
            widthMeters=0.6,
            direction=LaneDirection.ONE_WAY,
        )
    )
    zone = store.create_zone(
        ZoneCreate(
            name="Keep",
            type=ZoneType.KEEP_OUT,
            polygon=[
                ZonePoint(x=0.0, y=0.0),
                ZonePoint(x=1.0, y=0.0),
                ZonePoint(x=1.0, y=1.0),
            ],
        )
    )
    exported = store.export_map()
    assert exported.version == 2
    assert exported.frame_id == "map"
    assert len(exported.endpoints) == 1
    assert len(exported.lanes) == 1
    assert len(exported.zones) == 1

    other = SessionStore()
    other.import_map(exported.model_dump())
    assert other.get(ep.id) is not None
    assert other.get_lane(lane.id).direction == LaneDirection.ONE_WAY
    assert other.get_zone(zone.id).type == ZoneType.KEEP_OUT


def test_single_robot_default_capacity():
    store = SessionStore()
    z = store.create_zone(
        ZoneCreate(
            name="SR",
            type=ZoneType.SINGLE_ROBOT,
            polygon=[
                ZonePoint(x=0, y=0),
                ZonePoint(x=1, y=0),
                ZonePoint(x=0, y=1),
            ],
        )
    )
    assert z.capacity == 1


def test_import_replaces_session():
    store = SessionStore()
    store.create_lane(
        LaneCreate(
            name="Old",
            startX=0,
            startY=0,
            endX=1,
            endY=0,
            widthMeters=0.5,
            direction=LaneDirection.BIDIRECTIONAL,
        )
    )
    payload = MapVersion(
        version=2,
        frame_id="map",
        endpoints=[],
        lanes=[],
        zones=[],
    )
    store.import_map(payload)
    assert store.list_lanes() == []
