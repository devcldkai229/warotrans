from warotrans_fleet.sequence import SequenceClock


def test_sequences_start_at_one_and_increase():
    clock = SequenceClock(boot_id="boot-1")
    assert clock.next_heartbeat() == 1
    assert clock.next_heartbeat() == 2
    assert clock.next_telemetry() == 1
    assert clock.next_telemetry() == 2
    assert clock.next_telemetry() == 3


def test_heartbeat_and_telemetry_sequences_are_independent():
    clock = SequenceClock()
    clock.next_heartbeat()
    clock.next_heartbeat()
    assert clock.next_telemetry() == 1
    assert clock.heartbeat_sequence == 2


def test_boot_id_is_stable_for_process():
    clock = SequenceClock()
    first = clock.boot_id
    clock.next_heartbeat()
    assert clock.boot_id == first
