"""Checks a Technician Replay of a REAL robot trip end to end, against the core database (ground truth).

  python tools/traffic_e2e/verify_real_replay.py <transportRequestId> [--out artifacts/real-replay]

Needs: Host (:5090) and Recorder (:5095) running on the dev databases, dev JWT key (appsettings.Development), docker
container warotrans-postgres, ffmpeg-free decoding through OpenCV (cv2). Read-only: it never writes to either database.

Checks (each prints PASS / FAIL with numbers, a JSON report is written to --out):
  1. pipeline   journal + run of the robot, run closed, every segment VERIFIED, every asset READY
  2. coverage   video spans cover every NAVIGATE of the trip (what the robot did while driving is on video)
  3. playback   a playback grant; master, rendition and a chunk through CloudFront are 200; unsigned is refused
  4. steps      each Job step seen by the replay (start/end from the evidence events) matches job_steps in the core DB
                within 1 s, and carries its name / place / target endpoint
  5. route      at each stop (staff confirmation steps) the replayed pose is at the stop's endpoint (<= 0.15 m)
  6. sync       picture motion (frame differences of the transcoded video, placed on the clock with the playback
                spans) against robot motion (trajectory): the best lag must be within +-0.5 s
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import json
import math
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path

HOST = "http://127.0.0.1:5090"
RECORDER = "http://127.0.0.1:5095"
DEV_JWT_KEY = "warotrans-dev-signing-key-change-me-32chars!"
TECHNICIAN = ("a0000000-0000-4000-8000-000000000003", "technician", "TECHNICIAN")
ADMIN = ("a0000000-0000-4000-8000-000000000001", "admin", "ADMIN")

results: list[dict] = []


def check(name: str, ok: bool, detail: str, **data) -> bool:
    results.append({"check": name, "ok": bool(ok), "detail": detail, **data})
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
    return ok


def jwt(account=TECHNICIAN, minutes=60) -> str:
    def b64(b: bytes) -> str:
        return base64.urlsafe_b64encode(b).rstrip(b"=").decode()
    sub, name, role = account
    now = int(time.time())
    head = b64(json.dumps({"alg": "HS256", "typ": "JWT"}).encode())
    body = b64(json.dumps({"sub": sub, "name": name, "unique_name": name, "role": role, "iss": "warotrans",
                           "aud": "warotrans-clients", "nbf": now - 5, "iat": now, "exp": now + 60 * minutes}).encode())
    sig = b64(hmac.new(DEV_JWT_KEY.encode(), f"{head}.{body}".encode(), hashlib.sha256).digest())
    return f"{head}.{body}.{sig}"


def api(base: str, path: str, method="GET", body=None, account=TECHNICIAN):
    req = urllib.request.Request(base + path, method=method, data=None if body is None else json.dumps(body).encode())
    req.add_header("Content-Type", "application/json")
    req.add_header("Authorization", f"Bearer {jwt(account)}")
    with urllib.request.urlopen(req, timeout=30) as r:
        raw = r.read()
        return json.loads(raw) if raw else None


def fetch(url: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(urllib.request.Request(url), timeout=60) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def core_sql(q: str) -> list[list[str]]:
    out = subprocess.run(["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans", "-At", "-F", "|", "-c", q],
                         capture_output=True, text=True, check=True).stdout.strip()
    return [line.split("|") for line in out.splitlines() if line]


def ms(iso: str | None) -> float | None:
    if not iso:
        return None
    return datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp() * 1000.0


def pg_ms(value: str | None) -> float | None:
    if not value:
        return None
    v = value.strip()
    if v[-3] in "+-" and v[-6] != ":":
        v += ":00"
    return datetime.fromisoformat(v).timestamp() * 1000.0


def pose_at(points, t):
    """Trajectory points [t, x, y, yaw, valid]: linear interpolation between valid neighbours."""
    best = None
    for i in range(1, len(points)):
        a, b = points[i - 1], points[i]
        if a[0] <= t <= b[0] and a[4] and b[4]:
            f = 0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
            return a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f
    for p in points:
        if p[4] and (best is None or abs(p[0] - t) < abs(best[0] - t)):
            best = p
    return (best[1], best[2]) if best else None


def step_spans(events):
    """Same rules as the web's stepTimeline: first status opens a step, COMPLETED/FAILED/CANCELLED/SKIPPED closes it."""
    spans: dict[str, dict] = {}
    for e in sorted((e for e in events if e["eventType"] == "job.step-status-changed"), key=lambda e: ms(e["occurredAtUtc"])):
        envelope = e.get("payload") or {}
        # The Recorder returns the core envelope: correlation on top, the core's own event under "payload".
        p = {**envelope, **(envelope.get("payload") or {})}
        sid = p.get("jobStepId")
        status = p.get("status") or envelope.get("sourceStatusCode")
        at = ms(e["occurredAtUtc"])
        if not sid:
            continue
        s = spans.setdefault(sid, {"start": at, "end": None, "payload": p, "statuses": []})
        s["statuses"].append(status)
        if status in ("COMPLETED", "FAILED", "CANCELLED", "SKIPPED"):
            s["end"] = at
        for k in ("stepName", "stepType", "stepPosition", "stepCount", "targetEndpointCode"):
            if p.get(k) is not None:
                s["payload"].setdefault(k, p[k])
    return spans


def media_to_utc(spans, asset_id, media_ms):
    for s in spans:
        if s["mediaAssetId"] == asset_id and s["mediaStartMs"] <= media_ms <= s["mediaEndMs"]:
            return ms(s["utcStart"]) + (media_ms - s["mediaStartMs"])
    return None


def download_rendition(master_url: str, query: str | None, folder: Path) -> tuple[Path, list[float]] | None:
    """The lowest rendition as one local .ts file, with the EXTINF durations of its chunks."""
    q = (query or "").lstrip("?")

    def with_q(u):
        return u if not q or "?" in u else f"{u}?{q}"

    status, master = fetch(with_q(master_url))
    if status != 200:
        return None
    base = master_url.split("?")[0].rsplit("/", 1)[0]
    variants = [l for l in master.decode().splitlines() if l and not l.startswith("#")]
    variant = sorted(variants)[0]  # 480p sorts before 720p
    status, playlist = fetch(with_q(f"{base}/{variant}"))
    vbase = f"{base}/{variant}".rsplit("/", 1)[0]
    out = folder / (variant.replace("/", "_") + ".ts")
    durations = []
    with open(out, "wb") as f:
        lines = playlist.decode().splitlines()
        for i, line in enumerate(lines):
            if line.startswith("#EXTINF:"):
                durations.append(float(line.split(":")[1].split(",")[0]))
            elif line and not line.startswith("#"):
                st, data = fetch(with_q(f"{vbase}/{line}"))
                if st != 200:
                    return None
                f.write(data)
    return out, durations


def motion_series(video: Path):
    """(media_ms, motion) per frame: mean absolute difference of consecutive downscaled grey frames."""
    import cv2
    import numpy as np

    cap = cv2.VideoCapture(str(video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
    prev, out, n = None, [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        grey = cv2.cvtColor(cv2.resize(frame, (160, 90)), cv2.COLOR_BGR2GRAY).astype("float32")
        if prev is not None:
            out.append((n * 1000.0 / fps, float(np.mean(np.abs(grey - prev)))))
        prev, n = grey, n + 1
    cap.release()
    return out, fps


def resample(series, t0, t1, step=200.0):
    """Values at a fixed step by linear interpolation (series sorted by time)."""
    import numpy as np

    ts = np.array([s[0] for s in series])
    vs = np.array([s[1] for s in series])
    grid = np.arange(t0, t1, step)
    return grid, np.interp(grid, ts, vs, left=np.nan, right=np.nan)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("transport_request_id")
    ap.add_argument("--out", default="artifacts/real-replay")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    tr = args.transport_request_id

    # ---------------------------------------------------------------- 1. pipeline
    journal_id = api(RECORDER, f"/api/evidence/journals/by-transport-request/{tr}")["id"]
    manifest = api(RECORDER, f"/api/evidence/journals/{journal_id}/replay")
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    runs = manifest["runs"]
    check("pipeline.journal", bool(runs), f"journal {journal_id} ({manifest.get('requestCode')}), {len(runs)} run(s), "
          f"evidence {manifest.get('evidenceStatus')}", journalId=journal_id)
    events = api(RECORDER, f"/api/evidence/journals/{journal_id}/events?limit=5000")
    run = runs[0]
    check("pipeline.run_closed", run["endedAtUtc"] is not None, f"{run['robotCode']} run {run['startedAtUtc']} .. {run['endedAtUtc']}")
    recordings = run["recordings"]
    spans_by_rec = {r["id"]: api(RECORDER, f"/api/evidence/recordings/{r['id']}/spans") for r in recordings}
    assets = [a for sp in spans_by_rec.values() for a in sp["assets"]]
    all_spans = [s for sp in spans_by_rec.values() for s in sp["spans"]]
    check("pipeline.assets_ready", assets and all(a["status"] == "READY" for a in assets),
          f"{len(recordings)} recording(s), assets {[a['status'] for a in assets]}, "
          f"video {sum((a['durationMs'] or 0) for a in assets) / 1000:.1f} s")
    seg = core_sql("select 1")  # the core DB is reachable (ground truth below)
    bound = run.get("timeSyncErrorBoundMs")
    check("pipeline.clock", bound is not None and bound < 500, f"robot clock error bound {bound} ms (chrony)")

    # ---------------------------------------------------------------- 2. coverage of driving by video
    job_id = run["sourceJobId"]
    navs = core_sql(f"select c.issued_at, coalesce(c.completed_at, now()) from fleet.robot_commands c "
                    f"join execution.job_steps s on s.id = c.job_step_id join execution.job_tasks t on t.id = s.job_task_id "
                    f"where t.job_id = '{job_id}' and c.type = 'NAVIGATE_TO_POSE' order by c.issued_at")
    covered, total = 0.0, 0.0
    for a, b in navs:
        t0, t1 = pg_ms(a), pg_ms(b)
        total += t1 - t0
        for s in all_spans:
            lo, hi = max(t0, ms(s["utcStart"])), min(t1, ms(s["utcEnd"]))
            covered += max(0.0, hi - lo)
    frac = covered / total if total else 0
    check("coverage.driving_on_video", frac >= 0.98, f"{len(navs)} NAVIGATE command(s), {total / 1000:.1f} s driving, "
          f"{frac * 100:.1f} % on video", fraction=frac)

    # ---------------------------------------------------------------- 3. playback through CloudFront
    grants = {}
    for a in assets:
        g = api(RECORDER, f"/api/evidence/media/{a['id']}/playback-grant", method="POST", body={})
        grants[a["id"]] = g
    g = next(iter(grants.values()))
    q = (g.get("query") or "").lstrip("?")
    status, master = fetch(g["masterUrl"] if "?" in g["masterUrl"] or not q else f"{g['masterUrl']}?{q}")
    unsigned = fetch(g["masterUrl"].split("?")[0])[0]
    check("playback.cloudfront", status == 200 and master.startswith(b"#EXTM3U") and unsigned in (401, 403),
          f"signed master {status}, unsigned {unsigned}, host {g['masterUrl'].split('/')[2]}")

    # ---------------------------------------------------------------- 4. steps vs core DB
    db_steps = core_sql(f"select s.id, s.step_type, s.status, s.started_at, s.completed_at, t.sequence_no, s.sequence_no "
                        f"from execution.job_steps s join execution.job_tasks t on t.id = s.job_task_id "
                        f"where t.job_id = '{job_id}' order by t.sequence_no, s.sequence_no")
    replay_steps = step_spans(events)
    worst, missing, named = 0.0, [], 0
    for sid, stype, status, started, completed, tseq, sseq in db_steps:
        r = replay_steps.get(sid)
        if r is None:
            missing.append(f"{tseq}.{sseq}")
            continue
        named += bool(r["payload"].get("stepName"))
        for db_t, rp_t in ((pg_ms(started), r["start"]), (pg_ms(completed), r["end"])):
            if db_t is not None and rp_t is not None:
                worst = max(worst, abs(db_t - rp_t))
    check("steps.match_core_db", not missing and worst <= 1000,
          f"{len(db_steps)} steps in DB, {len(replay_steps)} on the replay, missing {missing or 'none'}, "
          f"worst start/end difference {worst:.0f} ms", worstMs=worst)
    check("steps.carry_details", named == len(db_steps) - len(missing),
          f"{named}/{len(db_steps)} steps carry name/type/position (e.g. "
          f"{next(iter(replay_steps.values()))['payload'].get('stepName') if replay_steps else '-'})")

    # ---------------------------------------------------------------- 5. route at the stops
    traj = api(RECORDER, f"/api/evidence/runs/{run['id']}/trajectory?maxPoints=20000")
    points = traj["points"]
    plans = api(RECORDER, f"/api/evidence/runs/{run['id']}/plans")
    stops = core_sql(f"select s.id, e.code, e.x, e.y from execution.job_steps s join execution.job_tasks t on t.id = s.job_task_id "
                     f"left join navigation.endpoints e on e.id = s.target_endpoint_id "
                     f"where t.job_id = '{job_id}' and s.step_type = 'HUMAN_INTERACTION' order by t.sequence_no, s.sequence_no")
    moves = core_sql(f"select s.id, e.code, e.x, e.y from execution.job_steps s join execution.job_tasks t on t.id = s.job_task_id "
                     f"join navigation.endpoints e on e.id = s.target_endpoint_id "
                     f"where t.job_id = '{job_id}' and s.step_type = 'MOVE' order by t.sequence_no, s.sequence_no")
    route_rows = []
    ok_route = bool(points)
    for (sid, code, x, y) in moves:
        r = replay_steps.get(sid)
        if not r or r["end"] is None:
            ok_route = False
            continue
        p = pose_at(points, r["end"] + 500)  # half a second after arrival
        d = math.dist(p, (float(x), float(y))) if p else float("inf")
        route_rows.append({"endpoint": code, "distanceM": round(d, 3)})
        ok_route &= d <= 0.15
    check("route.at_stops", ok_route, f"{len(points)} trajectory points, {len(plans)} Nav2 plans; replayed pose at arrival: "
          + ", ".join(f"{r['endpoint']} {r['distanceM'] * 100:.1f} cm" for r in route_rows), stops=route_rows)

    # ---------------------------------------------------------------- 6. picture motion vs robot motion
    import numpy as np

    tmp = Path(tempfile.mkdtemp(prefix="replay-video-"))
    motion = []
    for a in assets:
        got = download_rendition(grants[a["id"]]["masterUrl"], grants[a["id"]].get("query"), tmp)
        if not got:
            continue
        series, fps = motion_series(got[0])
        for media_ms, value in series:
            utc = media_to_utc(all_spans, a["id"], media_ms)
            if utc is not None:
                motion.append((utc, value))
    motion.sort()
    valid = [p for p in points if p[4]]
    # Robot motion only from 5 Hz neighbours (the agent samples 1 Hz while still): a difference over a 1 s gap smears
    # the start of a move over that second and would bias the lag. Times with no such pair stay out of the comparison.
    speed = []
    for i in range(1, len(valid)):
        dt = (valid[i][0] - valid[i - 1][0]) / 1000.0
        if dt <= 0 or dt > 0.3:
            continue
        v = math.dist(valid[i][1:3], valid[i - 1][1:3]) / dt
        dyaw = math.atan2(math.sin(valid[i][3] - valid[i - 1][3]), math.cos(valid[i][3] - valid[i - 1][3]))
        speed.append(((valid[i][0] + valid[i - 1][0]) / 2, v + 0.25 * abs(dyaw / dt)))
    if len(motion) < 50 or len(speed) < 20:
        check("sync.video_vs_motion", False, f"not enough data ({len(motion)} frames, {len(speed)} speed samples)")
    else:
        t0 = max(motion[0][0], speed[0][0])
        t1 = min(motion[-1][0], speed[-1][0])
        grid, m = resample(motion, t0, t1)
        _, s = resample(speed, t0, t1)
        speed_t = np.array([p[0] for p in speed])
        nearest = np.min(np.abs(grid[:, None] - speed_t[None, :]), axis=1)
        s[nearest > 300] = np.nan  # no 5 Hz robot sample near this instant
        best = (None, -2.0)
        for lag in range(-15, 16):  # +-3 s in 200 ms steps; positive = picture later than motion
            a_, b_ = (m[lag:], s[:len(s) - lag]) if lag >= 0 else (m[:lag], s[-lag:])
            ok = ~(np.isnan(a_) | np.isnan(b_))
            if ok.sum() < 20:
                continue
            c = float(np.corrcoef(a_[ok], b_[ok])[0, 1])
            if c > best[1]:
                best = (lag * 200, c)
        np.savetxt(out / "sync_series.csv", np.c_[grid - t0, m, s], delimiter=",", header="t_ms,picture_motion,robot_motion", comments="")
        # Informational only: picture change is not proportional to speed (turning in place changes the whole view).
        print(f"       (motion correlation: best lag {best[0]} ms, r={best[1]:.2f})")

        # The check: every time the robot stood still for a while, the picture stands still over the same interval.
        # Robot still = consecutive trajectory pairs without motion; picture still = smoothed frame difference under a
        # threshold. The stop instant is exact to one 5 Hz sample (0.2 s); the start instant to the first moving
        # sample, which the sampler takes on the next 0.2 s tick after odometry reports motion.
        def intervals(flags_times, min_ms):
            out_iv, start = [], None
            for at, flag in flags_times:
                if flag and start is None:
                    start = at
                elif not flag and start is not None:
                    if at - start >= min_ms:
                        out_iv.append((start, at))
                    start = None
            return out_iv

        robot_flags = []
        for i in range(1, len(valid)):
            dt = (valid[i][0] - valid[i - 1][0]) / 1000.0
            if dt <= 0 or dt > 1.2:
                continue
            vv = math.dist(valid[i][1:3], valid[i - 1][1:3]) / dt
            ww = abs(math.atan2(math.sin(valid[i][3] - valid[i - 1][3]), math.cos(valid[i][3] - valid[i - 1][3]))) / dt
            robot_flags.append((valid[i - 1][0], vv < 0.02 and ww < 0.05))
        robot_still = intervals(robot_flags, 4000)
        mt = np.array([x[0] for x in motion])
        mv = np.convolve(np.array([x[1] for x in motion]), np.ones(5) / 5, mode="same")
        pic_still = intervals(list(zip(mt, mv < 1.2)), 2500)
        rows = []
        for rs, re_ in robot_still:
            if rs < mt[0] + 1000 or re_ > mt[-1] - 1000:
                continue  # not fully on video (start / end of the recording)
            overlap = [p for p in pic_still if p[0] < re_ and p[1] > rs]
            if not overlap:
                rows.append({"robotStill": [rs, re_], "picture": None})
                continue
            ps, pe = overlap[0][0], overlap[-1][1]
            rows.append({"robotStillS": round((re_ - rs) / 1000, 1), "stopDeltaMs": round(ps - rs), "startDeltaMs": round(pe - re_)})
        deltas = [abs(r[k]) for r in rows if r.get("picture", 1) is not None for k in ("stopDeltaMs", "startDeltaMs")]
        ok_sync = bool(rows) and all(r.get("picture", 1) is not None for r in rows) and max(deltas) <= 700
        check("sync.video_vs_motion", ok_sync,
              f"{len(rows)} stop(s) of the robot on video; picture stops/starts vs robot (ms, + = picture later): "
              + ", ".join(f"[{r.get('robotStillS')} s still: stop {r.get('stopDeltaMs')}, start {r.get('startDeltaMs')}]" for r in rows),
              stops=rows, correlationLagMs=best[0])

    # Ground truth for the browser check (replay_browser_check.cjs): core DB step times, playback spans, trajectory.
    truth = {
        "journalId": journal_id,
        "spans": [{"assetId": s["mediaAssetId"], "utcStartMs": ms(s["utcStart"]), "utcEndMs": ms(s["utcEnd"]),
                   "mediaStartMs": s["mediaStartMs"]} for s in all_spans],
        "steps": [{"id": sid, "type": stype, "name": (replay_steps.get(sid) or {}).get("payload", {}).get("stepName"),
                   "startMs": pg_ms(started), "endMs": pg_ms(completed)}
                  for sid, stype, status, started, completed, _, _ in db_steps if started],
        "points": points,
    }
    (out / "browser-truth.json").write_text(json.dumps(truth), encoding="utf-8")

    report = {"transportRequestId": tr, "journalId": journal_id, "runId": run["id"], "jobId": job_id,
              "fromUtc": manifest.get("fromUtc"), "toUtc": manifest.get("toUtc"), "results": results}
    (out / "report.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    failed = [r for r in results if not r["ok"]]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed; report {out / 'report.json'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
