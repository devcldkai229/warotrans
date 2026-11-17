#!/usr/bin/env python3
"""Find the backend laptop (MQTT broker + Evidence Recorder) without a fixed IP, and follow it when its IP changes.

The robots and the laptop change addresses (laptop ICS DHCP, a different Wi-Fi in every room), so nothing on the
robot may hard-code one. Order of preference:

  1. BROKER given as an IP (manual override)              -> that IP
  2. a name (BROKER, else WARO_BACKEND_HOST, else the laptop's mDNS name) that resolves and answers on 1883
                                                           -> the NAME (paho / requests / chrony re-resolve it on
                                                              every reconnect, so a new laptop IP needs no restart)
  3. the laptop's ICS address 192.168.137.1, then a scan of every IPv4 /24 this robot is on
                                                           -> the IP of the host with 1883 open (5095 too preferred)

    broker_discovery.py resolve [--name NAME] [--retry-s 60]   print the backend, exit 3 when none
    broker_discovery.py check TARGET                         exit 0 when TARGET answers on 1883
    broker_discovery.py watch --state FILE --on-change CMD   every 30 s: when the target in FILE stops answering
                                                             3 times, resolve again; a different answer runs CMD
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Iterable, Optional

DEFAULT_HOST = "LAPTOP-Q0QAV6GM.local"
ICS_GATEWAY = "192.168.137.1"
MQTT_PORT = 1883
RECORDER_PORT = 5095

Probe = Callable[[str, int, float], bool]


def tcp_open(host: str, port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def is_ip(value: str) -> bool:
    try:
        ipaddress.ip_address(value)
        return True
    except ValueError:
        return False


def resolves(name: str) -> bool:
    try:
        return bool(socket.getaddrinfo(name, MQTT_PORT, socket.AF_INET, socket.SOCK_STREAM))
    except OSError:
        return False


def local_networks() -> list[tuple[str, ipaddress.IPv4Network]]:
    """(own address, its /24) of every global IPv4 address of this machine (ip -o -4 addr)."""
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show", "scope", "global"], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_ip_addr(out)


def parse_ip_addr(out: str) -> list[tuple[str, ipaddress.IPv4Network]]:
    nets = []
    for line in out.splitlines():
        parts = line.split()
        if "inet" not in parts:
            continue
        iface = ipaddress.ip_interface(parts[parts.index("inet") + 1])
        # Only the /24 around the robot: a /16 hotspot would take minutes; the laptop is next to the robot anyway.
        nets.append((str(iface.ip), ipaddress.ip_network(f"{iface.ip}/24", strict=False)))
    return nets


def scan(networks: Iterable[tuple[str, ipaddress.IPv4Network]], probe: Probe = tcp_open, timeout: float = 0.3) -> Optional[str]:
    """The host with 1883 open; among several, the one that also runs the Recorder (5095)."""
    own = set()
    candidates: list[str] = []
    for addr, net in networks:
        own.add(addr)
        candidates.extend(str(h) for h in net.hosts())
    candidates = [c for c in dict.fromkeys(candidates) if c not in own]
    if not candidates:
        return None
    with ThreadPoolExecutor(max_workers=64) as pool:
        hits = [c for c, ok in zip(candidates, pool.map(lambda c: probe(c, MQTT_PORT, timeout), candidates)) if ok]
    if not hits:
        return None
    with_recorder = [h for h in hits if probe(h, RECORDER_PORT, timeout)]
    return (with_recorder or hits)[0]


def resolve_once(broker: str = "", name: str = "", probe: Probe = tcp_open, has_name: Callable[[str], bool] = resolves,
                 networks: Callable[[], list] = local_networks) -> Optional[str]:
    broker = broker.strip()
    if broker and broker.lower() != "auto" and is_ip(broker):
        return broker
    names = [n for n in dict.fromkeys([broker if broker.lower() not in ("", "auto") else "", name, DEFAULT_HOST]) if n]
    for n in names:
        if has_name(n) and probe(n, MQTT_PORT, 1.5):
            return n
    if probe(ICS_GATEWAY, MQTT_PORT, 0.5):
        return ICS_GATEWAY
    return scan(networks(), probe)


def resolve(broker: str = "", name: str = "", retry_s: float = 0.0, sleep: Callable[[float], None] = time.sleep,
            clock: Callable[[], float] = time.monotonic, **kw) -> Optional[str]:
    deadline = clock() + retry_s
    while True:
        found = resolve_once(broker, name, **kw)
        if found or clock() >= deadline:
            return found
        sleep(5.0)


def watch_step(current: str, misses: int, probe: Probe = tcp_open, **kw) -> tuple[int, Optional[str]]:
    """One watcher round: (misses, new target or None). A target is only replaced after 3 missed checks in a row."""
    if current and current != "none" and probe(current, MQTT_PORT, 1.5):
        return 0, None
    misses += 1
    if current not in ("", "none") and misses < 3:
        return misses, None
    found = resolve_once(probe=probe, **kw)
    if found and found != current:
        return 0, found
    return misses, None


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("resolve")
    r.add_argument("--broker", default=os.environ.get("BROKER", ""))
    r.add_argument("--name", default=os.environ.get("WARO_BACKEND_HOST", ""))
    r.add_argument("--retry-s", type=float, default=0.0)
    c = sub.add_parser("check")
    c.add_argument("target")
    w = sub.add_parser("watch")
    w.add_argument("--state", required=True, help="file holding the target the stack was started with")
    w.add_argument("--on-change", required=True, help="shell command run once the target changed (restarts the stack)")
    w.add_argument("--name", default=os.environ.get("WARO_BACKEND_HOST", ""))
    w.add_argument("--interval-s", type=float, default=30.0)
    args = ap.parse_args(argv)

    if args.cmd == "resolve":
        found = resolve(args.broker, args.name, args.retry_s)
        if not found:
            print("no backend found (MQTT 1883) by name, at 192.168.137.1 or on the local /24", file=sys.stderr)
            return 3
        print(found)
        return 0
    if args.cmd == "check":
        return 0 if tcp_open(args.target, MQTT_PORT, 1.5) else 1

    misses = 0
    while True:
        time.sleep(args.interval_s)
        try:
            with open(args.state, encoding="utf-8") as f:
                current = f.read().strip()
        except OSError:
            continue  # the stack has not started yet
        misses, new = watch_step(current, misses, name=args.name)
        if new:
            print(f"backend moved: {current or 'none'} -> {new}; restarting the stack", flush=True)
            subprocess.run(args.on_change, shell=True, check=False)
            misses = 0
            time.sleep(60)  # let the restart write the new state


if __name__ == "__main__":
    sys.exit(main())
