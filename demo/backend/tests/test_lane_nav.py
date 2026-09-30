"""Unit tests for Phase 3 lane pose sampling."""

from app.lane_nav import build_lane_sequence_poses, sample_lane_centerline
from app.session import (
    Lane,
    LaneDirection,
    TrafficZone,
    ZonePoint,
    ZoneType,
)


def test_sample_lane_spacing():
    lane = Lane(
        id="1",
        name="A",
        startX=0.0,
        startY=0.0,
        endX=1.0,
        endY=0.0,
        widthMeters=0.6,
        direction=LaneDirection.ONE_WAY,
    )
    poses = sample_lane_centerline(lane, spacing_m=0.35)
    assert poses[0]["x"] == 0.0
    assert poses[-1]["x"] == 1.0
    assert len(poses) >= 3


def test_chain_through_junction_gap():
    a = Lane(
        id="a",
        name="A",
        startX=0.0,
        startY=0.0,
        endX=1.0,
        endY=0.0,
        widthMeters=0.5,
        direction=LaneDirection.ONE_WAY,
    )
    b = Lane(
        id="b",
        name="B",
        startX=1.2,
        startY=0.0,
        endX=2.0,
        endY=0.0,
        widthMeters=0.5,
        direction=LaneDirection.ONE_WAY,
    )
    j = TrafficZone(
        id="j",
        name="J",
        type=ZoneType.JUNCTION,
        polygon=[
            ZonePoint(x=0.8, y=-0.3),
            ZonePoint(x=1.4, y=-0.3),
            ZonePoint(x=1.4, y=0.3),
            ZonePoint(x=0.8, y=0.3),
        ],
    )
    poses = build_lane_sequence_poses([a, b], junctions=[j])
    assert poses[-1]["x"] == 2.0
    assert len(poses) > 4
