from dataclasses import dataclass, field
from typing import Optional

import pytest
from traffic_fakes import Clock, make_authority
from warotrans_fleet import command_contract as cc
from warotrans_fleet.route_grant import GraphEdge
from warotrans_fleet.safety_supervisor import (
    STATUS_FAULT,
    STATUS_IDLE,
    STATUS_OK,
    STATUS_PAUSED,
    SafetyConfig,
    SafetySupervisor,
)

EDGES = [GraphEdge(1, 0.0, 0.0, 2.0, 0.0), GraphEdge(2, 2.0, 0.0, 4.0, 0.0), GraphEdge(9, 0.0, 5.0, 4.0, 5.0)]


@dataclass
class World:
    clock: Clock = field(default_factory=Clock)
    motion: bool = True
    authority: Optional[object] = None
    pose: Optional[tuple] = (1.0, 0.0)
    pose_age: Optional[float] = 0.05
    scan_age: Optional[float] = 0.05
    localized: bool = True
    progress: Optional[float] = 0.0
    last_authority_at: Optional[float] = None
    next_edge: Optional[int] = None
    faults: list = field(default_factory=list)
    ok_stream: list = field(default_factory=list)


def make(world: World, **cfg) -> SafetySupervisor:
    world.authority = world.authority or make_authority(edges=(1, 2))
    world.last_authority_at = world.clock.now
    return SafetySupervisor(
        config=SafetyConfig(**cfg),
        authority_provider=lambda: world.authority,
        motion_expected=lambda: world.motion,
        pose_provider=lambda: world.pose,
        pose_age_provider=lambda: world.pose_age,
        scan_age_provider=lambda: world.scan_age,
        localization_ok_provider=lambda: world.localized,
        progress_provider=lambda: world.progress,
        last_authority_at_provider=lambda: world.last_authority_at,
        next_edge_provider=lambda: world.next_edge,
        edges=EDGES,
        on_fault=lambda code, detail: world.faults.append(code),
        on_authority_ok=world.ok_stream.append,
        clock=world.clock,
        wall_clock=world.clock,
    )


def test_ok_when_everything_healthy():
    w = World()
    sup = make(w)
    status = sup.tick()
    assert (status.authority_ok, status.state) == (True, STATUS_OK)
    assert sup.authority_ok is True and w.ok_stream == [True]


def test_idle_when_no_motion_expected_and_no_faults():
    w = World(motion=False)
    sup = make(w)
    status = sup.tick()
    assert (status.authority_ok, status.state) == (False, STATUS_IDLE)
    assert w.faults == []


@pytest.mark.parametrize(
    "mutate,reason",
    [
        (lambda w: setattr(w, "pose_age", 0.9), "POSE_STALE"),
        (lambda w: setattr(w, "pose", None), "POSE_STALE"),
        (lambda w: setattr(w, "scan_age", 0.9), "SCAN_STALE"),
        (lambda w: setattr(w, "scan_age", None), "SCAN_STALE"),
        (lambda w: setattr(w, "localized", False), "LOCALIZATION_LOST"),
        (lambda w: setattr(w, "next_edge", 4), "NEXT_EDGE_NOT_GRANTED"),
    ],
)
def test_operational_pause_is_recoverable_and_not_a_fault(mutate, reason):
    w = World()
    sup = make(w)
    mutate(w)
    status = sup.tick()
    assert status.state == STATUS_PAUSED and status.authority_ok is False
    assert reason in (status.reason or "")
    assert w.faults == [] and sup.latched_fault is None

    # hết điều kiện → tự hồi phục
    w2 = World()
    sup2 = make(w2)
    mutate(w2)
    sup2.tick()
    w2.pose, w2.pose_age, w2.scan_age, w2.localized, w2.next_edge = (1.0, 0.0), 0.05, 0.05, True, None
    assert sup2.tick().authority_ok is True


def test_next_granted_edge_is_fine():
    w = World(next_edge=2)
    assert make(w).tick().authority_ok is True


def test_pause_escalates_to_fault_after_timeout():
    w = World()
    sup = make(w, pause_fault_timeout_sec=2.0)
    w.localized = False
    assert sup.tick().state == STATUS_PAUSED
    w.clock.advance(2.0)
    status = sup.tick()
    assert status.state == STATUS_FAULT and status.fault_code == cc.ERROR_LOCALIZATION_LOST
    assert w.faults == [cc.ERROR_LOCALIZATION_LOST]

    w2 = World()
    sup2 = make(w2, pause_fault_timeout_sec=2.0)
    w2.scan_age = 5.0
    sup2.tick()
    w2.clock.advance(2.0)
    assert sup2.tick().fault_code == cc.ERROR_SAFETY_STOP


def test_pause_that_clears_resets_escalation_timer():
    w = World()
    sup = make(w, pause_fault_timeout_sec=2.0)
    w.scan_age = 5.0
    sup.tick()
    w.clock.advance(1.5)
    w.scan_age = 0.05
    assert sup.tick().authority_ok is True
    w.scan_age = 5.0
    w.clock.advance(1.5)
    assert sup.tick().state == STATUS_PAUSED  # timer bắt đầu lại


def test_pause_escalation_can_be_disabled():
    w = World()
    sup = make(w, pause_fault_timeout_sec=0.0)
    w.localized = False
    sup.tick()
    w.clock.advance(1000)
    w.last_authority_at = w.clock.now  # backend vẫn renew → watchdog không nổ
    assert sup.tick().state == STATUS_PAUSED and w.faults == []


def test_authority_expired_is_emergency_fault():
    w = World(authority=make_authority(edges=(1, 2), expires_at=1000.0 + 1))
    sup = make(w)
    assert sup.tick().authority_ok is True
    w.clock.advance(1.0)
    status = sup.tick()
    assert (status.state, status.fault_code) == (STATUS_FAULT, cc.ERROR_AUTHORITY_EXPIRED)
    assert w.faults == [cc.ERROR_AUTHORITY_EXPIRED]


def test_missing_authority_while_moving_is_fault():
    faults = []
    sup = SafetySupervisor(
        authority_provider=lambda: None,
        motion_expected=lambda: True,
        on_fault=lambda code, detail: faults.append(code),
    )
    assert sup.tick().fault_code == cc.ERROR_AUTHORITY_LOST
    assert faults == [cc.ERROR_AUTHORITY_LOST]


def test_corridor_violation_is_emergency_fault():
    w = World(pose=(1.0, 0.2))  # trong corridor 0.35 m
    sup = make(w, corridor_buffer_m=0.35)
    assert sup.tick().authority_ok is True
    w.pose = (1.0, 0.5)
    status = sup.tick()
    assert (status.state, status.fault_code) == (STATUS_FAULT, cc.ERROR_CORRIDOR_VIOLATION)


def test_corridor_ignores_ungranted_edges():
    w = World(pose=(1.0, 5.0))  # đang nằm trên edge 9 (không được cấp)
    sup = make(w)
    assert sup.tick().fault_code == cc.ERROR_CORRIDOR_VIOLATION


def test_corridor_not_evaluated_with_stale_pose_only_pauses():
    w = World(pose=(1.0, 9.0), pose_age=2.0)
    sup = make(w)
    status = sup.tick()
    assert status.state == STATUS_PAUSED and w.faults == []


def test_mqtt_authority_watchdog_faults_authority_lost():
    w = World()
    sup = make(w, authority_loss_timeout_sec=3.0)
    w.clock.advance(2.9)
    assert sup.tick().authority_ok is True
    w.clock.advance(0.2)
    status = sup.tick()
    assert (status.state, status.fault_code) == (STATUS_FAULT, cc.ERROR_AUTHORITY_LOST)
    # renew (last_authority_at cập nhật) sẽ giữ được ok
    w2 = World()
    sup2 = make(w2, authority_loss_timeout_sec=3.0)
    w2.clock.advance(2.9)
    w2.last_authority_at = w2.clock.now
    w2.clock.advance(2.9)
    assert sup2.tick().authority_ok is True


def test_no_progress_faults_controller_failed():
    w = World()
    sup = make(w, progress_timeout_sec=5.0, progress_epsilon=0.02)
    assert sup.tick().authority_ok is True  # khởi tạo tracker
    w.clock.advance(4.0)
    w.last_authority_at = w.clock.now
    w.progress = 0.01  # dưới epsilon
    assert sup.tick().authority_ok is True
    w.clock.advance(1.5)
    w.last_authority_at = w.clock.now
    status = sup.tick()
    assert (status.state, status.fault_code) == (STATUS_FAULT, cc.ERROR_CONTROLLER_FAILED)


def test_progress_resets_stall_timer():
    w = World()
    sup = make(w, progress_timeout_sec=5.0)
    sup.tick()
    for step in range(1, 6):
        w.clock.advance(3.0)
        w.last_authority_at = w.clock.now
        w.progress = 0.1 * step
        assert sup.tick().authority_ok is True


def test_pause_does_not_count_toward_stall():
    w = World()
    sup = make(w, progress_timeout_sec=5.0, pause_fault_timeout_sec=0.0)
    sup.tick()
    w.scan_age = 9.0
    for _ in range(4):
        w.clock.advance(3.0)
        w.last_authority_at = w.clock.now
        assert sup.tick().state == STATUS_PAUSED
    w.scan_age = 0.05
    assert sup.tick().authority_ok is True  # tracker bắt đầu lại, chưa stall


def test_fault_latches_until_motion_restarts_after_idle():
    w = World(authority=make_authority(edges=(1, 2), expires_at=1000.0 + 1))
    sup = make(w)
    w.clock.advance(1.0)
    assert sup.tick().state == STATUS_FAULT
    # client chưa rời motion → vẫn latched, authority_ok=False
    w.authority = make_authority(edges=(1, 2), seq=2)
    assert sup.tick().state == STATUS_FAULT and sup.authority_ok is False
    # client vào FAULTED (motion False) rồi nhận motion mới → xóa latch
    w.motion = False
    assert sup.tick().state == STATUS_FAULT
    w.motion = True
    w.last_authority_at = w.clock.now
    assert sup.tick().authority_ok is True and sup.latched_fault is None


def test_stopped_never_reports_ok_or_release():
    w = World(motion=False)
    sup = make(w)
    for _ in range(3):
        assert sup.tick().authority_ok is False
    assert w.ok_stream == [False, False, False]
    assert not hasattr(sup, "release")
