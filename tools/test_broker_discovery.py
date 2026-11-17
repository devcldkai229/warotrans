"""broker_discovery: the robot finds the backend by name, at the ICS address or by a scan, and follows it."""

from __future__ import annotations

import ipaddress
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import broker_discovery as bd  # noqa: E402


def probe_of(open_ports: dict[str, set[int]]):
    return lambda host, port, timeout: port in open_ports.get(host, set())


NO_NETS = lambda: []  # noqa: E731


def test_an_explicit_ip_is_used_as_given():
    assert bd.resolve_once("10.0.0.9", probe=probe_of({}), has_name=lambda n: False, networks=NO_NETS) == "10.0.0.9"


def test_a_name_that_resolves_and_answers_is_kept_as_a_name():
    probe = probe_of({"LAPTOP-Q0QAV6GM.local": {1883}})
    assert bd.resolve_once("auto", probe=probe, has_name=lambda n: True, networks=NO_NETS) == "LAPTOP-Q0QAV6GM.local"


def test_a_configured_name_comes_before_the_default():
    probe = probe_of({"lab-laptop.local": {1883}, "LAPTOP-Q0QAV6GM.local": {1883}})
    assert bd.resolve_once("", name="lab-laptop.local", probe=probe, has_name=lambda n: True, networks=NO_NETS) == "lab-laptop.local"


def test_without_mdns_the_ics_address_then_the_scan_find_it():
    assert bd.resolve_once("", probe=probe_of({"192.168.137.1": {1883}}), has_name=lambda n: False, networks=NO_NETS) == "192.168.137.1"
    nets = lambda: [("192.168.11.40", ipaddress.ip_network("192.168.11.0/24"))]  # noqa: E731
    probe = probe_of({"192.168.11.7": {1883}, "192.168.11.201": {1883, 5095}, "192.168.11.40": {1883}})
    # the robot's own address is skipped; the host that also runs the Recorder wins
    assert bd.resolve_once("", probe=probe, has_name=lambda n: False, networks=nets) == "192.168.11.201"


def test_nothing_found_is_none():
    nets = lambda: [("192.168.11.40", ipaddress.ip_network("192.168.11.0/24"))]  # noqa: E731
    assert bd.resolve_once("auto", probe=probe_of({}), has_name=lambda n: False, networks=nets) is None


def test_resolve_retries_until_the_deadline():
    t = [0.0]
    calls = []

    def probe(host, port, timeout):
        calls.append(host)
        return t[0] >= 10 and host == "192.168.137.1"

    found = bd.resolve("", retry_s=30, sleep=lambda s: t.__setitem__(0, t[0] + s), clock=lambda: t[0],
                       probe=probe, has_name=lambda n: False, networks=NO_NETS)
    assert found == "192.168.137.1"


def test_parse_ip_addr_takes_each_global_ipv4_as_its_24():
    out = ("2: eth0    inet 192.168.137.50/24 brd 192.168.137.255 scope global eth0\\       valid_lft forever\n"
           "3: wlan0    inet 192.168.11.40/22 brd 192.168.11.255 scope global dynamic wlan0\n")
    nets = bd.parse_ip_addr(out)
    assert nets == [("192.168.137.50", ipaddress.ip_network("192.168.137.0/24")),
                    ("192.168.11.40", ipaddress.ip_network("192.168.11.0/24"))]


def test_watch_keeps_a_working_target_and_replaces_a_dead_one_after_three_misses():
    alive = probe_of({"192.168.11.201": {1883}})
    assert bd.watch_step("192.168.11.201", 0, probe=alive, has_name=lambda n: False, networks=NO_NETS) == (0, None)

    moved = probe_of({"192.168.137.1": {1883}})
    misses, new = bd.watch_step("192.168.11.201", 0, probe=moved, has_name=lambda n: False, networks=NO_NETS)
    assert (misses, new) == (1, None)
    misses, new = bd.watch_step("192.168.11.201", misses, probe=moved, has_name=lambda n: False, networks=NO_NETS)
    assert (misses, new) == (2, None)
    assert bd.watch_step("192.168.11.201", misses, probe=moved, has_name=lambda n: False, networks=NO_NETS) == (0, "192.168.137.1")


def test_watch_starts_the_link_as_soon_as_a_backend_appears_after_a_start_without_one():
    probe = probe_of({"192.168.137.1": {1883}})
    assert bd.watch_step("none", 0, probe=probe, has_name=lambda n: False, networks=NO_NETS) == (0, "192.168.137.1")


def test_watch_does_not_restart_for_the_same_name():
    # mDNS hiccup: the name stops answering, then resolves to itself again -> no restart
    calls = {"n": 0}

    def probe(host, port, timeout):
        calls["n"] += 1
        return calls["n"] > 3 and host == "LAPTOP-Q0QAV6GM.local"

    misses = 0
    for _ in range(3):
        misses, new = bd.watch_step("LAPTOP-Q0QAV6GM.local", misses, probe=probe, has_name=lambda n: True, networks=NO_NETS)
        assert new is None
