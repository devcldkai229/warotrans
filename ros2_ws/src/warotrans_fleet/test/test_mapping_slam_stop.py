"""Stopping slam_toolbox waits for the whole process group, not only for `ros2 launch` (lab 2026-11-25)."""

from warotrans_fleet.mapping_ros import wait_group_gone


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


def test_returns_once_the_last_process_of_the_group_exited():
    clock = Clock()
    alive_until = 1.0

    def kill(_pgid, _sig):
        if clock.t >= alive_until:
            raise ProcessLookupError

    assert wait_group_gone(42, 8.0, kill=kill, sleep=clock.sleep, clock=clock)
    assert 1.0 <= clock.t < 1.2


def test_gives_up_after_the_timeout_when_a_process_lingers():
    clock = Clock()
    assert not wait_group_gone(42, 2.0, kill=lambda *_: None, sleep=clock.sleep, clock=clock)
    assert clock.t >= 2.0
