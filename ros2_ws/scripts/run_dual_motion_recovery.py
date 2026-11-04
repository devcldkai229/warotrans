#!/usr/bin/env python3
"""Stage-3 dual live motion recovery scenarios A–F (Parking A/B starts)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


def yaw_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class NsNav(Node):
    def __init__(self, ns: str) -> None:
        super().__init__(
            f"recovery_nav_{ns}",
            namespace=ns,
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
        )
        self.ns = ns
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tfl = TransformListener(self.tf, self, spin_thread=True)
        self.map_frame = "map"
        self.base_frame = f"{ns}/base_footprint"

    def pose(self) -> tuple[float, float] | None:
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def navigate(self, x: float, y: float, yaw: float, timeout: float) -> tuple[str, int]:
        if not self.ac.wait_for_server(timeout_sec=20.0):
            return "NO_SERVER", -1
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        z, w = yaw_quat(yaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        t0 = time.time()
        send = self.ac.send_goal_async(goal)
        while not send.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send.done() or send.result() is None or not send.result().accepted:
            return "REJECTED", -1
        res_fut = send.result().get_result_async()
        while not res_fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res_fut.done():
            try:
                send.result().cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            return "TIMEOUT", -1
        wrap = res_fut.result()
        status = int(wrap.status)
        code = int(getattr(wrap.result, "error_code", 0))
        if status == 4:
            return "PASS", code
        return f"FAIL_status_{status}", code


def announce(msg: str) -> None:
    print(f"\n=== ANNOUNCE: {msg} ===\n", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoints", required=True)
    ap.add_argument("--out-csv", required=True)
    ap.add_argument("--out-md", required=True)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument(
        "--tests",
        default="A,B,C,D,E,F",
        help="Comma list of test ids to run (stop early on safety FAIL if needed)",
    )
    args = ap.parse_args()
    eps = {e["code"]: e for e in json.loads(Path(args.endpoints).read_text(encoding="utf-8"))["endpoints"]}
    want = {t.strip().upper() for t in args.tests.split(",") if t.strip()}

    need_sim01 = bool(want & {"A", "C", "D", "E", "F"})
    need_sim02 = bool(want & {"B", "C", "D", "E", "F"})

    rclpy.init()
    sim01 = NsNav("sim01") if need_sim01 else None
    sim02 = NsNav("sim02") if need_sim02 else None
    t0 = time.time()
    while time.time() - t0 < 20.0:
        if sim01 is not None:
            rclpy.spin_once(sim01, timeout_sec=0.05)
        if sim02 is not None:
            rclpy.spin_once(sim02, timeout_sec=0.05)
        ok1 = (sim01 is None) or (sim01.pose() is not None)
        ok2 = (sim02 is None) or (sim02.pose() is not None)
        if ok1 and ok2:
            break
    announce(
        f"TF ready SIM-01={None if sim01 is None else sim01.pose()} "
        f"(start EP-PARKING-A when present) "
        f"SIM-02={None if sim02 is None else sim02.pose()} "
        f"(start EP-PARKING-B when present)"
    )

    rows: list[dict] = []
    md = [
        "# Dual motion recovery run",
        "",
        f"**Updated:** {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "**Starts:** SIM-01 EP-PARKING-A · SIM-02 EP-PARKING-B",
        "**Mode:** Semantic Traffic · no STAGE1_OPEN_GRANT",
        "",
    ]

    def run_leg(
        test_id: str,
        robot: str,
        node: NsNav,
        start_code: str,
        goal_code: str,
        *,
        note: str = "",
    ) -> dict:
        g = eps[goal_code]
        announce(
            f"Test {test_id}: robot={robot} START={start_code} → GOAL={goal_code} "
            f"({g['x']:.3f},{g['y']:.3f}) {note}"
        )
        start = node.pose()
        status, code = node.navigate(float(g["x"]), float(g["y"]), float(g.get("yaw") or 0.0), args.timeout)
        end = node.pose()
        ge = None if end is None else dist(end, (float(g["x"]), float(g["y"])))
        disp = None
        if start and end:
            disp = dist(start, end)
        row = {
            "test": test_id,
            "robot": robot,
            "start_endpoint": start_code,
            "goal_endpoint": goal_code,
            "status": status,
            "error_code": code,
            "start_xy": "" if start is None else f"{start[0]:.3f},{start[1]:.3f}",
            "end_xy": "" if end is None else f"{end[0]:.3f},{end[1]:.3f}",
            "displacement_m": "" if disp is None else f"{disp:.3f}",
            "goal_error_m": "" if ge is None else f"{ge:.3f}",
            "note": note,
        }
        rows.append(row)
        md.append(
            f"- **{test_id}/{robot}** `{start_code}`→`{goal_code}`: **{status}** "
            f"disp={row['displacement_m']} err={row['goal_error_m']} code={code}"
        )
        print(row, flush=True)
        return row

    stop_later = False

    # A — SIM-01 only
    if "A" in want and not stop_later:
        assert sim01 is not None
        r = run_leg("A", "SIM-01", sim01, "EP-PARKING-A", "EP-INBOUND", note="standalone semantic")
        if r["status"] != "PASS":
            stop_later = True
            md.append("- Safety gate: Test A FAIL → later hazardous tests marked NOT_RUN/blocked")

    # B — SIM-02 only
    if "B" in want and not stop_later:
        assert sim02 is not None
        r = run_leg("B", "SIM-02", sim02, "EP-PARKING-B", "EP-OUTBOUND", note="standalone semantic")
        if r["status"] != "PASS":
            stop_later = True
            md.append("- Safety gate: Test B FAIL → stop later dual contention")

    # C — concurrent non-overlapping corridors (geometry-checked approx)
    # SIM-01 ParkingA → INBOUND (east-north); SIM-02 ParkingB → OUTBOUND (east-south)
    if "C" in want and not stop_later:
        announce(
            "Test C: concurrent independent — SIM-01 EP-PARKING-A→EP-INBOUND "
            "AND SIM-02 EP-PARKING-B→EP-OUTBOUND"
        )
        g1, g2 = eps["EP-INBOUND"], eps["EP-OUTBOUND"]
        # Launch SIM-01 then SIM-02 without waiting (overlap)
        # Sequential wait with shared timeout budget via threaded-ish polling:
        t_deadline = time.time() + args.timeout
        # send both goals by calling navigate sequentially with remaining time is too serialized;
        # use short first-accept then poll — simplified: navigate sim01 with timeout/2 while
        # we cannot truly parallel without multi-executor; run SIM-01 then SIM-02 is NOT C.
        # Use async goals:
        def send(node: NsNav, g: dict):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(g["x"])
            goal.pose.pose.position.y = float(g["y"])
            z, w = yaw_quat(float(g.get("yaw") or 0.0))
            goal.pose.pose.orientation.z = z
            goal.pose.pose.orientation.w = w
            if not node.ac.wait_for_server(timeout_sec=10.0):
                return None
            return node.ac.send_goal_async(goal)

        s1 = sim01.pose()
        s2 = sim02.pose()
        f1 = send(sim01, g1)
        f2 = send(sim02, g2)
        while time.time() < t_deadline and (f1 is None or not f1.done() or f2 is None or not f2.done()):
            rclpy.spin_once(sim01, timeout_sec=0.05)
            rclpy.spin_once(sim02, timeout_sec=0.05)
        if f1 is None or f2 is None or not f1.done() or not f2.done():
            rows.append(
                {
                    "test": "C",
                    "robot": "BOTH",
                    "start_endpoint": "EP-PARKING-A/B",
                    "goal_endpoint": "EP-INBOUND/EP-OUTBOUND",
                    "status": "REJECTED",
                    "error_code": -1,
                    "start_xy": "",
                    "end_xy": "",
                    "displacement_m": "",
                    "goal_error_m": "",
                    "note": "goal accept timeout",
                }
            )
            stop_later = True
        else:
            h1, h2 = f1.result(), f2.result()
            r1 = h1.get_result_async() if h1.accepted else None
            r2 = h2.get_result_async() if h2.accepted else None
            while time.time() < t_deadline:
                rclpy.spin_once(sim01, timeout_sec=0.05)
                rclpy.spin_once(sim02, timeout_sec=0.05)
                if (r1 is None or r1.done()) and (r2 is None or r2.done()):
                    break
            def finish(label, handle, fut, start_xy, goal_code, g):
                end = sim01.pose() if label == "SIM-01" else sim02.pose()
                if handle is None or not handle.accepted:
                    st, code = "REJECTED", -1
                elif fut is None or not fut.done():
                    st, code = "TIMEOUT", -1
                else:
                    wrap = fut.result()
                    code = int(getattr(wrap.result, "error_code", 0))
                    st = "PASS" if int(wrap.status) == 4 else f"FAIL_status_{wrap.status}"
                disp = "" if start_xy is None or end is None else f"{dist(start_xy, end):.3f}"
                ge = "" if end is None else f"{dist(end, (float(g['x']), float(g['y']))):.3f}"
                row = {
                    "test": "C",
                    "robot": label,
                    "start_endpoint": "EP-PARKING-A" if label == "SIM-01" else "EP-PARKING-B",
                    "goal_endpoint": goal_code,
                    "status": st,
                    "error_code": code,
                    "start_xy": "" if start_xy is None else f"{start_xy[0]:.3f},{start_xy[1]:.3f}",
                    "end_xy": "" if end is None else f"{end[0]:.3f},{end[1]:.3f}",
                    "displacement_m": disp,
                    "goal_error_m": ge,
                    "note": "concurrent independent",
                }
                rows.append(row)
                md.append(
                    f"- **C/{label}** →`{goal_code}`: **{st}** disp={disp} err={ge}"
                )
                print(row, flush=True)
                return st == "PASS"

            ok1 = finish("SIM-01", h1 if h1.accepted else None, r1, s1, "EP-INBOUND", g1)
            ok2 = finish("SIM-02", h2 if h2.accepted else None, r2, s2, "EP-OUTBOUND", g2)
            if not (ok1 and ok2):
                stop_later = True

    # D — SINGLE_OCC contention: both toward same protected center approach (INBOUND)
    if "D" in want and not stop_later:
        announce(
            "Test D: SINGLE_OCC contention — both robots → EP-INBOUND (capacity arbitration)"
        )
        g = eps["EP-INBOUND"]

        def mk_goal(node: NsNav, gg: dict):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(gg["x"])
            goal.pose.pose.position.y = float(gg["y"])
            z, w = yaw_quat(float(gg.get("yaw") or 0.0))
            goal.pose.pose.orientation.z = z
            goal.pose.pose.orientation.w = w
            return goal

        s1, s2 = sim01.pose(), sim02.pose()
        fut1 = sim01.ac.send_goal_async(mk_goal(sim01, g))
        fut2 = sim02.ac.send_goal_async(mk_goal(sim02, g))
        deadline = time.time() + args.timeout
        while time.time() < deadline and (not fut1.done() or not fut2.done()):
            rclpy.spin_once(sim01, timeout_sec=0.05)
            rclpy.spin_once(sim02, timeout_sec=0.05)
        if not fut1.done() or not fut2.done():
            rows.append(
                {
                    "test": "D",
                    "robot": "BOTH",
                    "start_endpoint": "current",
                    "goal_endpoint": "EP-INBOUND",
                    "status": "REJECTED",
                    "error_code": -1,
                    "start_xy": "",
                    "end_xy": "",
                    "displacement_m": "",
                    "goal_error_m": "",
                    "note": "contention accept timeout",
                }
            )
            stop_later = True
        else:
            h1, h2 = fut1.result(), fut2.result()
            r1 = h1.get_result_async() if h1.accepted else None
            r2 = h2.get_result_async() if h2.accepted else None
            while time.time() < deadline:
                rclpy.spin_once(sim01, timeout_sec=0.05)
                rclpy.spin_once(sim02, timeout_sec=0.05)
                if (r1 is None or r1.done()) and (r2 is None or r2.done()):
                    break
            for label, node, handle, fut, start in (
                ("SIM-01", sim01, h1, r1, s1),
                ("SIM-02", sim02, h2, r2, s2),
            ):
                end = node.pose()
                if not handle.accepted:
                    st, code = "REJECTED", -1
                elif fut is None or not fut.done():
                    st, code = "TIMEOUT", -1
                else:
                    wrap = fut.result()
                    code = int(getattr(wrap.result, "error_code", 0))
                    st = "PASS" if int(wrap.status) == 4 else f"FAIL_status_{wrap.status}"
                row = {
                    "test": "D",
                    "robot": label,
                    "start_endpoint": "current",
                    "goal_endpoint": "EP-INBOUND",
                    "status": st,
                    "error_code": code,
                    "start_xy": "" if start is None else f"{start[0]:.3f},{start[1]:.3f}",
                    "end_xy": "" if end is None else f"{end[0]:.3f},{end[1]:.3f}",
                    "displacement_m": ""
                    if start is None or end is None
                    else f"{dist(start, end):.3f}",
                    "goal_error_m": ""
                    if end is None
                    else f"{dist(end, (float(g['x']), float(g['y']))):.3f}",
                    "note": "SINGLE_OCC same goal",
                }
                rows.append(row)
                md.append(f"- **D/{label}** →EP-INBOUND: **{st}**")
                print(row, flush=True)

    # E — unmarked intersection (cross: SIM-01→OUTBOUND, SIM-02→INBOUND from wherever)
    if "E" in want and not stop_later:
        announce("Test E: unmarked crossing — SIM-01→EP-OUTBOUND, SIM-02→EP-SHELF-A-01")
        # Use sequential announce; parallel goals
        g1, g2 = eps["EP-OUTBOUND"], eps["EP-SHELF-A-01"]

        def mk(node, gg):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(gg["x"])
            goal.pose.pose.position.y = float(gg["y"])
            z, w = yaw_quat(float(gg.get("yaw") or 0.0))
            goal.pose.pose.orientation.z = z
            goal.pose.pose.orientation.w = w
            return goal

        s1, s2 = sim01.pose(), sim02.pose()
        fut1 = sim01.ac.send_goal_async(mk(sim01, g1))
        fut2 = sim02.ac.send_goal_async(mk(sim02, g2))
        deadline = time.time() + args.timeout
        while time.time() < deadline and (not fut1.done() or not fut2.done()):
            rclpy.spin_once(sim01, timeout_sec=0.05)
            rclpy.spin_once(sim02, timeout_sec=0.05)
        h1, h2 = fut1.result(), fut2.result()
        r1 = h1.get_result_async() if h1.accepted else None
        r2 = h2.get_result_async() if h2.accepted else None
        while time.time() < deadline:
            rclpy.spin_once(sim01, timeout_sec=0.05)
            rclpy.spin_once(sim02, timeout_sec=0.05)
            if (r1 is None or r1.done()) and (r2 is None or r2.done()):
                break
        for label, node, handle, fut, start, code_ep, gg in (
            ("SIM-01", sim01, h1, r1, s1, "EP-OUTBOUND", g1),
            ("SIM-02", sim02, h2, r2, s2, "EP-SHELF-A-01", g2),
        ):
            end = node.pose()
            if not handle.accepted:
                st, code = "REJECTED", -1
            elif fut is None or not fut.done():
                st, code = "TIMEOUT", -1
            else:
                wrap = fut.result()
                code = int(getattr(wrap.result, "error_code", 0))
                st = "PASS" if int(wrap.status) == 4 else f"FAIL_status_{wrap.status}"
            row = {
                "test": "E",
                "robot": label,
                "start_endpoint": "current",
                "goal_endpoint": code_ep,
                "status": st,
                "error_code": code,
                "start_xy": "" if start is None else f"{start[0]:.3f},{start[1]:.3f}",
                "end_xy": "" if end is None else f"{end[0]:.3f},{end[1]:.3f}",
                "displacement_m": ""
                if start is None or end is None
                else f"{dist(start, end):.3f}",
                "goal_error_m": ""
                if end is None
                else f"{dist(end, (float(gg['x']), float(gg['y']))):.3f}",
                "note": "unmarked cross",
            }
            rows.append(row)
            md.append(f"- **E/{label}** →{code_ep}: **{st}**")
            print(row, flush=True)

    # F — failure/recovery smoke (localization revoke is backend unit-tested; here: cancel + re-nav)
    if "F" in want and not stop_later:
        announce("Test F: cancel mid-nav then re-grant SIM-01 → EP-WAIT-POINT (stale plan / replan)")
        g = eps.get("EP-WAIT-POINT") or eps["EP-INSPECTION"]
        # Start goal then cancel quickly, then re-send
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = sim01.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(g["x"])
        goal.pose.pose.position.y = float(g["y"])
        z, w = yaw_quat(float(g.get("yaw") or 0.0))
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        fut = sim01.ac.send_goal_async(goal)
        t_end = time.time() + 15.0
        while time.time() < t_end and not fut.done():
            rclpy.spin_once(sim01, timeout_sec=0.05)
        if fut.done() and fut.result().accepted:
            fut.result().cancel_goal_async()
            time.sleep(2.0)
        r = run_leg(
            "F",
            "SIM-01",
            sim01,
            "current",
            g.get("code") if isinstance(g.get("code"), str) else "EP-WAIT-POINT",
            note="after cancel / replan",
        )
        # Override goal code if WAIT-POINT missing key
        if "EP-WAIT-POINT" not in eps and rows:
            rows[-1]["goal_endpoint"] = "EP-INSPECTION"

    # Mark skipped
    for tid in sorted(want):
        if not any(r["test"] == tid for r in rows):
            rows.append(
                {
                    "test": tid,
                    "robot": "-",
                    "start_endpoint": "-",
                    "goal_endpoint": "-",
                    "status": "NOT_RUN" if stop_later else "NOT_RUN",
                    "error_code": "",
                    "start_xy": "",
                    "end_xy": "",
                    "displacement_m": "",
                    "goal_error_m": "",
                    "note": "blocked by earlier safety gate" if stop_later else "not executed",
                }
            )
            md.append(f"- **{tid}**: NOT_RUN")

    out_csv = Path(args.out_csv)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "test",
        "robot",
        "start_endpoint",
        "goal_endpoint",
        "status",
        "error_code",
        "start_xy",
        "end_xy",
        "displacement_m",
        "goal_error_m",
        "note",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow(row)
    Path(args.out_md).write_text("\n".join(md) + "\n", encoding="utf-8")
    announce(f"DONE wrote {out_csv}")
    if sim01 is not None:
        sim01.destroy_node()
    if sim02 is not None:
        sim02.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
