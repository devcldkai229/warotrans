"""Lab script 2026-11-26 on MAP-ONE-V027 with the two REAL robots (happy cases H1-H7).

Every scenario creates a real transport request (with its stock voucher for INBOUND / OUTBOUND) that the Host sends to
the robots; person steps waiting at the same moment are confirmed together (real_two_robot_run.confirm_waiting).
The V027 shelves start empty, so the order feeds the later scenarios: inbound first, outbound last.

    python tools/traffic_e2e/real_v027_run.py H1 H2 H3 H4 H5 H6 H7
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import real_two_robot_run as R  # noqa: E402
from real_two_robot_run import OUT, World, _home_after, _pair, _pick, _queue_all, create_tr, new_containers, voucher  # noqa: E402


def shelves(w: World) -> dict[str, str]:
    """'A-1' -> storage location code on V027 (SHELF-A-1262611 ...)."""
    out = {}
    for loc, ep in w.loc_ep.items():
        if ep.startswith("EP-SHELF-"):
            out[ep[len("EP-SHELF-"):]] = loc
    return out


def _inbound(w: World, dst: list[str], tag: str, qi: bool = False) -> list[str]:
    S = shelves(w)
    boxes = new_containers(w, len(dst), tag)
    voucher(w, "INBOUND", boxes, f"lab V027 {tag}")
    moves = [{"container": c, "src": "EP-INBOUND", "dst": f"EP-SHELF-{d}", "dstLoc": S[d]} for c, d in zip(boxes, dst)]
    return create_tr(w, "INBOUND", moves, f"lab V027 {tag}", qi)[1]


def _reloc(w: World, pairs: list[tuple[str, str]], tag: str, used: set) -> list[str]:
    S = shelves(w)
    moves = [{"container": _pick(w, S[s], 1, used)[0], "src": f"EP-SHELF-{s}", "srcLoc": S[s], "dst": f"EP-SHELF-{d}", "dstLoc": S[d]}
             for s, d in pairs]
    return create_tr(w, "INTERNAL_RELOCATION", moves, f"lab V027 {tag}")[1]


def _outbound(w: World, src: list[str], tag: str, used: set, qi: bool = False) -> list[str]:
    S = shelves(w)
    boxes = [(_pick(w, S[s], 1, used)[0], s) for s in src]
    voucher(w, "OUTBOUND", [c for c, _ in boxes], f"lab V027 {tag}")
    moves = [{"container": c, "src": f"EP-SHELF-{s}", "srcLoc": S[s], "dst": "EP-OUTBOUND"} for c, s in boxes]
    return create_tr(w, "OUTBOUND_RETRIEVAL", moves, f"lab V027 {tag}", qi)[1]


def H1(w):
    """1 INBOUND request, 6 boxes, 2 per shelf, through QUALITY-INSPECTION: two trips of 3, both robots."""
    return _home_after(w, "H1", _queue_all(w, "H1", _inbound(w, ["A-1", "A-1", "A-2", "A-2", "B-1", "B-1"], "H1", qi=True)))


def H2(w):
    """1 INTERNAL_RELOCATION request, 6 boxes, 2 to each shelf."""
    used: set = set()
    return _home_after(w, "H2", _queue_all(w, "H2", _reloc(w, [("A-1", "B-2"), ("A-1", "B-2"), ("A-2", "A-1"), ("A-2", "A-1"),
                                                                ("B-1", "A-2"), ("B-1", "A-2")], "H2", used)))


def H3(w):
    """INBOUND - INBOUND at the same moment, 2 boxes each."""
    return _pair(w, "H3", [("RBT-001", _inbound(w, ["B-1", "B-1"], "H3-RBT-001")),
                           ("RBT-002", _inbound(w, ["A-2", "A-1"], "H3-RBT-002"))])


def H4(w):
    """INBOUND (RBT-001) - INTERNAL_RELOCATION (RBT-002) at the same moment, 2 boxes each."""
    used: set = set()
    return _pair(w, "H4", [("RBT-001", _inbound(w, ["B-2", "B-2"], "H4-RBT-001")),
                           ("RBT-002", _reloc(w, [("A-1", "B-1"), ("A-1", "B-1")], "H4-RBT-002", used))])


def H5(w):
    """Two independent requests sent together: INBOUND (2) on RBT-002, OUTBOUND (2) on RBT-001."""
    used: set = set()
    return _pair(w, "H5", [("RBT-002", _inbound(w, ["A-2", "A-2"], "H5-RBT-002")),
                           ("RBT-001", _outbound(w, ["B-2", "B-2"], "H5-RBT-001", used))])


def H6(w):
    """INTERNAL_RELOCATION (RBT-001) - OUTBOUND (RBT-002) at the same moment, 2 boxes each."""
    used: set = set()
    return _pair(w, "H6", [("RBT-001", _reloc(w, [("B-1", "A-1"), ("B-1", "A-1")], "H6-RBT-001", used)),
                           ("RBT-002", _outbound(w, ["A-2", "A-2"], "H6-RBT-002", used))])


def H7(w):
    """1 OUTBOUND request, 6 boxes, 2 from each shelf, through QUALITY-INSPECTION."""
    used: set = set()
    return _home_after(w, "H7", _queue_all(w, "H7", _outbound(w, ["A-1", "A-1", "A-2", "A-2", "B-1", "B-1"], "H7", used, qi=True)))


CASES = {"H1": H1, "H2": H2, "H3": H3, "H4": H4, "H5": H5, "H6": H6, "H7": H7}


def main(names: list[str]) -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    w = World()
    print("map", w.map_id[:8], "shelves", shelves(w), flush=True)
    summary = {}
    for name in names:
        print(f"\n===== {name}  (RBT-001 near {R.nearest(w, 'RBT-001')}, RBT-002 near {R.nearest(w, 'RBT-002')})", flush=True)
        res = CASES[name](w)
        simultaneous = sum(1 for c in res.get("confirmations", []) if c.get("together"))
        summary[name] = {"pass": res["pass"], "seconds": res["seconds"], "minBodyGapM": res["minBodyGapM"],
                         "parallelPct": res["parallelPct"], "simultaneousConfirms": simultaneous,
                         "jobs": {k: f"{v['status']} {v['robot']}" for k, v in res["jobs"].items()}}
        print(f"##### {name} {'PASS' if res['pass'] else 'FAIL'} {json.dumps(summary[name])}", flush=True)
        if not res["pass"]:
            break
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "summary_v027.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print("\nSUMMARY", json.dumps(summary, indent=1), flush=True)
    return 0 if all(v["pass"] for v in summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or list(CASES)))
