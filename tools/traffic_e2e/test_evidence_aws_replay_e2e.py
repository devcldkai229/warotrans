"""Technician Replay on real AWS, end to end: one trip of a simulated robot, filmed, uploaded, transcoded and played back.

  real Host (Semantic traffic, test DB) + SimRobot over MQTT: one request, one container, staff confirmations
    -> EvidenceBridge pushes the real transport events to the Recorder (journal + run of the robot assignment)
  fake camera (real JPEG frames showing the robot's pose) -> real RecorderSink (AVI / MJPEG segments)
    -> real AgentCore + Spool + Uploader with the robot credential
    -> Recorder (Storage S3, Queues SQS): presigned PUT straight to S3 raw/
    -> S3 notification -> SQS segment-ingest -> Worker verifies size + SHA-256, seals the group
    -> SQS media-process -> AWS Elemental MediaConvert (HLS 720p + 480p) -> EventBridge -> SQS media-result
    -> Worker: HLS check, playback spans, asset READY
  -> replay API + CloudFront signed playback URL: the master playlist, a rendition and a segment are fetched.

Costs a little (S3, SQS, a few seconds of MediaConvert): it only runs with WARO_E2E_AWS=1. AWS settings come from the
dotnet user-secrets of the Recorder and the Worker (Development), never from this file. Own *test* databases.

  WARO_E2E_AWS=1 WAROTRANS_SYSTEM_REPO=<system checkout> python -m pytest tools/traffic_e2e/test_evidence_aws_replay_e2e.py -q -s

Leaves artifacts/aws-replay-summary.json (journal id, request id, asset id) for the UI check.
"""

from __future__ import annotations

import io
import json
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import asdict
from pathlib import Path

import pytest

AWS = os.environ.get("WARO_E2E_AWS") == "1"
pytestmark = pytest.mark.skipif(not AWS, reason="real AWS: set WARO_E2E_AWS=1 (S3, SQS, MediaConvert, CloudFront)")

# Own evidence database: never the developer's warotrans_evidence (the harness refuses names without "test").
os.environ.setdefault("WARO_E2E_EVIDENCE_DB", "warotrans_evidence_aws_e2e_test")

from harness import (ROBOT_REPO, Api, EvidenceRecorder, EvidenceWorker, Host, mint_jwt,  # noqa: E402
                     wait_for)
from map_paths import MapPaths  # noqa: E402
from test_frontend_flow import _sql  # noqa: E402
from test_traffic_e2e import ARTIFACTS, PREFIX, RUN, SimRobot  # noqa: E402

# The map published in the dev database today (the seeder copies it into warotrans_test by code).
BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / os.environ.get("WARO_E2E_MAP_DIR", "map_one_v013")
MANIFEST = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
ZONES = json.loads((BUNDLE / "single_occupancy.json").read_text(encoding="utf-8"))["zones"]
MAP_ID, MAP_CODE, FINGERPRINT = MANIFEST["mapVersionId"], MANIFEST["mapCode"], MANIFEST["semanticMapFingerprint"]
PATHS = MapPaths(BUNDLE)
HERE = Path(__file__).resolve().parent


def _seed(args: list[str]) -> None:
    import subprocess

    done = subprocess.run([sys.executable, str(HERE / "seed_test_db.py"), *args], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr

sys.path.insert(0, str(ROBOT_REPO / "ros2_ws" / "src" / "warotrans_camera"))
sys.path.insert(0, str(ROBOT_REPO / "ros2_ws" / "src" / "warotrans_evidence_agent"))

from warotrans_camera.recorder_sink import RecorderSink  # noqa: E402
from warotrans_evidence_agent.agent_core import AgentCore, CameraProfile  # noqa: E402
from warotrans_evidence_agent.recorder_api import RecorderApi  # noqa: E402
from warotrans_evidence_agent.spool import Spool  # noqa: E402
from warotrans_evidence_agent.uploader import Uploader  # noqa: E402

ROBOT = f"E2E-{RUN}-AWS"
WIDTH, HEIGHT, FPS, SEGMENT_S = 640, 480, 10, 4.0
LOCAL_ONLY = ("Evidence__Storage__Provider", "Evidence__Storage__LocalRoot", "Evidence__Queues__Provider",
              "Evidence__Storage__LocalPublicBaseUrl", "Evidence__Storage__LocalSigningKey")


class AwsRecorder(EvidenceRecorder):
    """The harness Recorder with its user-secrets providers (S3, SQS, CloudFront) instead of the Local ones."""

    def env(self) -> dict[str, str]:
        e = super().env()
        for key in LOCAL_ONLY:
            e.pop(key, None)
        e["Evidence__Playback__GrantMode"] = "Query"  # *.cloudfront.net: signed query string, no cookie domain
        e["Evidence__VideoRequired"] = "true"
        return e


class AwsWorker(EvidenceWorker):
    """The harness Worker with its user-secrets providers (S3, SQS, MediaConvert)."""

    def start(self) -> None:
        saved = {k: os.environ.pop(k) for k in LOCAL_ONLY if k in os.environ}
        import harness

        original = harness.evidence_shared_env
        harness.evidence_shared_env = lambda: {k: v for k, v in original().items() if k not in LOCAL_ONLY}
        try:
            super().start()
        finally:
            harness.evidence_shared_env = original
            os.environ.update(saved)


class JpegCamera:
    """A camera source for RecorderSink: real JPEG frames at FPS showing where the simulated robot is."""

    def __init__(self, robot: SimRobot, bounds: tuple[float, float, float, float]) -> None:
        import PIL.Image  # noqa: F401  (fail early when Pillow is missing)

        self.robot, self.bounds = robot, bounds
        self.frame, self.seq, self.mono, self.wall = None, 0, 0, 0
        self.viewers = 0
        self.trail: list[tuple[float, float]] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="jpeg-camera")

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(5)

    def add_viewer(self) -> None:
        self.viewers += 1

    def remove_viewer(self) -> None:
        self.viewers -= 1

    def latest(self):
        with self._lock:
            return self.frame, self.seq, self.mono, self.wall

    def _to_px(self, x: float, y: float) -> tuple[int, int]:
        x0, y0, x1, y1 = self.bounds
        px = 40 + (x - x0) / max(1e-6, x1 - x0) * (WIDTH - 80)
        py = HEIGHT - 60 - (y - y0) / max(1e-6, y1 - y0) * (HEIGHT - 120)
        return int(px), int(py)

    def _render(self) -> bytes:
        from PIL import Image, ImageDraw

        img = Image.new("RGB", (WIDTH, HEIGHT), (24, 28, 36))
        draw = ImageDraw.Draw(img)
        for i in range(0, WIDTH, 40):
            draw.line([(i, 0), (i, HEIGHT)], fill=(36, 42, 54))
        for j in range(0, HEIGHT, 40):
            draw.line([(0, j), (WIDTH, j)], fill=(36, 42, 54))
        x, y, yaw = self.robot.x, self.robot.y, self.robot.yaw
        self.trail.append((x, y))
        self.trail = self.trail[-400:]
        if len(self.trail) > 1:
            draw.line([self._to_px(*p) for p in self.trail], fill=(80, 160, 255), width=3)
        cx, cy = self._to_px(x, y)
        draw.ellipse([cx - 12, cy - 12, cx + 12, cy + 12], fill=(255, 196, 0))
        import math

        draw.line([(cx, cy), (cx + int(22 * math.cos(yaw)), cy - int(22 * math.sin(yaw)))], fill=(255, 255, 255), width=3)
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()) + f".{int(time.time() * 10) % 10} UTC"
        draw.rectangle([0, 0, WIDTH, 34], fill=(10, 12, 16))
        draw.text((10, 10), f"{ROBOT}  x={x:5.2f} y={y:5.2f}  frame {self.seq}", fill=(230, 230, 230))
        draw.text((WIDTH - 230, 10), stamp, fill=(160, 200, 255))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()

    def _run(self) -> None:
        period = 1.0 / FPS
        next_at = time.monotonic()
        while not self._stop.is_set():
            frame = self._render()
            with self._lock:
                self.seq += 1
                self.frame, self.mono, self.wall = frame, time.monotonic_ns(), time.time_ns()
            next_at += period
            time.sleep(max(0.0, next_at - time.monotonic()))


class RobotSide:
    """What runs on the Pi, in one thread like the agent node: camera control, active run, pose, upload."""

    def __init__(self, api: RecorderApi, camera: JpegCamera, robot: SimRobot, spool_dir: Path) -> None:
        self.api, self.camera, self.robot = api, camera, robot
        self.spool = Spool(str(spool_dir / "evidence.db"))
        self.segments: "queue.Queue[dict]" = queue.Queue()
        self.sink = RecorderSink(camera, lambda info: self.segments.put(asdict(info)), WIDTH, HEIGHT, FPS, SEGMENT_S)
        self.core = AgentCore(self.spool, str(spool_dir), CameraProfile(width=WIDTH, height=HEIGHT, fps=FPS, nominal_fov_degrees=120,
                                                                         camera_model="E2E synthetic camera"),
                              str(uuid.uuid4()), self._control, map_version=MAP_CODE)
        self.uploader = Uploader(self.spool, api, rate_limit_bps=None, delete_uploaded=True)
        self.recording_keys: list[str] = []
        self.runs_seen: set[str] = set()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="robot-side")
        self.last_error = ""

    def _control(self, message: dict) -> None:
        if message["action"] == "start":
            self.recording_keys.append(message["recordingKey"])
            self.sink.start(message["recordingKey"], message["directory"])
        elif message["action"] == "stop":
            self.sink.stop()

    def start(self) -> None:
        self._thread.start()

    def _drain(self) -> None:
        while True:
            try:
                self.core.on_camera_segment(self.segments.get_nowait())
            except queue.Empty:
                return

    def _loop(self) -> None:
        last_poll = 0.0
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                if now - last_poll >= 1.0:
                    last_poll = now
                    run = self.api.active_run()
                    if run:
                        self.runs_seen.add(run["runId"])
                    self.core.on_active_run(run)
                self.core.on_pose(self.robot.x, self.robot.y, self.robot.yaw, 0.3, 0.0, 0.05, True)
                self._drain()
                self.core.tick()
                self.uploader.pump(max_uploads=4)
            except Exception as exc:  # keep going like the node does; the test asserts on the outcome
                self.last_error = str(exc)
            time.sleep(0.25)

    def finish(self, timeout: float = 120.0) -> None:
        """The trip is over: stop recording, flush telemetry, upload everything that is left."""
        self._stop.set()
        self._thread.join(10)
        self.core.on_active_run(None)
        self.core.shutdown()
        self.sink.stop()
        self._drain()
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._drain()
            self.uploader.pump(max_uploads=10)
            done = all(self.spool.recording(k).state == "FINALIZED" for k in self.recording_keys)
            if self.spool.pending_upload_count() == 0 and self.spool.outbox_size() == 0 and done:
                return
            time.sleep(0.5)
        raise AssertionError(f"robot side did not finish uploading: {self.uploader.last_error or self.last_error}")


def evidence_sql(query: str) -> list:
    from seed_test_db import psql
    from harness import EVIDENCE_DB

    return json.loads(psql(EVIDENCE_DB, f"SELECT coalesce(jsonb_agg(t), '[]'::jsonb) FROM ({query}) t;") or "[]")


def fetch(url: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.status, resp.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def test_a_filmed_trip_is_transcoded_on_aws_and_plays_back_through_cloudfront(tmp_path):
    ARTIFACTS.mkdir(exist_ok=True)
    _seed(["--cleanup-e2e-robots"])
    # Containers an earlier run put on a shelf point at the warehouse the seeder replaces: let go of it first.
    _sql("update warehouse.containers set inventory_storage_location_id = null, inventory_level_no = null, warehouse_id = null "
         "where warehouse_id in (select warehouse_id from navigation.map_versions) returning id")
    _seed(["--map-code", MAP_CODE, "--workflow-code", "INTERNAL_RELOCATION"])
    recorder = AwsRecorder(ARTIFACTS)
    worker = AwsWorker(ARTIFACTS, port=5197)  # the default (Recorder + 1 = 5199) is the harness Host's port
    host = Host(ARTIFACTS, name="replay-aws", extra_env={
        **recorder.host_env(),
        f"Traffic__RobotHomeEndpoints__{ROBOT}": "EP-PARKING-A",
        "JobPlanning__MaxContainersPerJob": "1",
        # As the RealRobot profile.
        "Traffic__SemanticMapDirectory": str(BUNDLE),
        "Traffic__SemanticPrefixGrant": "true",
        "Traffic__SemanticCellSizeMeters": "0.10",
    })
    sim = camera = side = None
    recorder.start()
    try:
        worker.start()
        host.start()
        api = Api(host.base)
        api.login()

        eps = {c: (i, float(x), float(y)) for c, i, x, y in
               _sql(f"select code, id, x, y from navigation.endpoints where map_version_id = '{MAP_ID}'")}
        wid = _sql(f"select warehouse_id from navigation.map_versions where id = '{MAP_ID}'")[0][0]
        src, dst, park = eps["EP-SHELF-A-1"], eps["EP-SHELF-B-2"], eps["EP-PARKING-A"]
        sim = SimRobot(code=ROBOT, x=park[1], y=park[2], fingerprint=FINGERPRINT, zones=ZONES, prefix=PREFIX,
                       map_code=MAP_CODE, path_provider=PATHS.provider(), speed=0.3, progress_timeout_sec=60.0)
        rid = api.call("POST", "/api/fleet/robots", {"warehouseId": wid, "currentMapVersionId": MAP_ID, "code": ROBOT,
                                                     "name": ROBOT})["id"]
        sim.start()
        wait_for(lambda: (lambda r: r.get("isOnline") and r.get("localizationStatus") == "LOCALIZED")(
            api.call("GET", f"/api/fleet/robots/{rid}")), 15, what="online and localized")
        api.call("POST", f"/api/fleet/robots/{rid}/put-in-service", {})

        # The robot's own recording credential, issued by an admin of the Recorder.
        admin = Api(recorder.base)
        admin.token = mint_jwt("ADMIN")
        token = admin.call("POST", f"/api/evidence/admin/robots/{ROBOT}/credentials")["token"]
        edge = RecorderApi(recorder.base, ROBOT, token)
        assert edge.whoami()["robotCode"] == ROBOT

        xs = [v[1] for v in eps.values()]
        ys = [v[2] for v in eps.values()]
        camera = JpegCamera(sim, (min(xs) - 0.3, min(ys) - 0.3, max(xs) + 0.3, max(ys) + 0.3))
        camera.start()
        side = RobotSide(edge, camera, sim, tmp_path / "spool")
        side.start()

        cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"AW-{RUN}", "name": "E2E AWS"})["id"]
        prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"AW-{RUN}", "name": "E2E AWS"})["id"]
        # A relocation between two shelves (an inbound put-away would need a confirmed voucher first).
        def shelf_at(endpoint_id: str, label: str) -> str:
            # One storage location per endpoint: reuse the one an earlier run left in warotrans_test.
            found = _sql(f"select id from warehouse.storage_locations where endpoint_id = '{endpoint_id}'")
            if found and found[0] and found[0][0]:
                return found[0][0]
            return api.call("POST", "/api/warehouse/storage-locations", {
                "warehouseId": wid, "endpointId": endpoint_id, "code": f"SL-AW-{RUN}-{label}", "name": f"E2E shelf {label} {RUN}"})["id"]

        shelf_from, shelf_to = shelf_at(src[0], "A"), shelf_at(dst[0], "B")
        container = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
        # On the shelf already (test database only): the inbound voucher and put-away are not what this test is about.
        _sql(f"update warehouse.containers set status = 'STORED', current_storage_location_id = '{shelf_from}', "
             f"current_level_no = 1, inventory_storage_location_id = '{shelf_from}', inventory_level_no = 1, "
             f"warehouse_id = '{wid}' where id = '{container}' returning id")
        wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows?pageSize=100")["items"]
                  if w["code"] == "INTERNAL_RELOCATION")["id"]
        tr = api.call("POST", "/api/transportation/transport-requests", {
            "warehouseId": wid, "mapVersionId": MAP_ID, "workflowId": wf, "workflowInputs": {}, "note": "aws replay e2e",
            "details": [{"sequenceNo": 1, "containerId": container,
                         "sourceEndpointId": src[0], "sourceStorageLocationId": shelf_from, "sourceLevelNo": 1,
                         "destinationEndpointId": dst[0], "destinationStorageLocationId": shelf_to, "destinationLevelNo": 1}]})["id"]
        job = api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})["jobs"][0]["jobId"]
        assert api.call("POST", f"/api/workflow-execution/jobs/{job}/start", {})["status"] == "RUNNING"

        def waiting():
            j = api.call("GET", f"/api/workflow-execution/jobs/{job}")
            if j["status"] != "RUNNING":
                return j
            for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
                for s in t["steps"]:
                    if s["status"] == "WAITING":
                        return s
            return None

        confirmed = []
        while True:
            s = wait_for(waiting, 180, interval=0.5, what="next confirmation")
            if "tasks" in s:
                assert s["status"] == "COMPLETED", s.get("failureMessage")
                break
            time.sleep(3.0)  # staff take a moment at the stop: the video shows the robot standing there
            api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm", {"containerId": container, "note": "e2e"})
            confirmed.append(s["operationCode"])
        assert confirmed[0] == "PICKUP_CONFIRM" and confirmed[-1] == "DROPOFF_CONFIRM", confirmed
        wait_for(lambda: api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED", 30,
                 what="request completed")

        time.sleep(3.0)
        side.finish()
        assert side.runs_seen, f"the robot never saw its run: {side.last_error}"
        assert side.recording_keys

        # The journal of the request, with the robot's run.
        journal = wait_for(lambda: recorder.journal(api, tr), 60, interval=1.0, what="journal")
        journal_id = journal["id"]
        keys = ",".join(f"'{k}'" for k in side.recording_keys)
        total = evidence_sql(f"SELECT count(*) AS n FROM evidence.video_segments s JOIN evidence.camera_recordings r "
                             f"ON r.id = s.recording_id WHERE r.edge_recording_key IN ({keys})")[0]["n"]
        assert total >= 3

        # S3 → SQS → Worker: every file verified (size + SHA-256), then one MediaConvert job per continuous group.
        wait_for(lambda: evidence_sql(f"SELECT count(*) AS n FROM evidence.video_segments s JOIN evidence.camera_recordings r "
                                      f"ON r.id = s.recording_id WHERE r.edge_recording_key IN ({keys}) "
                                      f"AND s.upload_status = 'VERIFIED'")[0]["n"] == total,
                 180, interval=3.0, what="segments verified from S3 notifications")

        def assets():
            return evidence_sql(f"SELECT a.id, a.status, j.status AS job, j.provider_job_id, j.last_error_code, j.last_error_message FROM evidence.media_assets a "
                                f"JOIN evidence.camera_recordings r ON r.id = a.recording_id "
                                f"LEFT JOIN evidence.media_processing_jobs j ON j.media_asset_id = a.id "
                                f"WHERE r.edge_recording_key IN ({keys})")

        def settled():
            rows = assets()
            if rows and all(r["status"] in ("READY", "FAILED") for r in rows):
                return rows
            return None

        rows = wait_for(settled, 900, interval=10.0, what="MediaConvert result (asset READY or FAILED)")
        print("media assets:", json.dumps(rows, indent=1))
        assert all(r["status"] == "READY" for r in rows), rows

        # The replay a technician opens, and the video through CloudFront.
        reader = Api(recorder.base)
        reader.token = api.token
        replay = reader.call("GET", f"/api/evidence/journals/{journal_id}/replay")
        Path(ARTIFACTS / "aws-replay.json").write_text(json.dumps(replay, indent=1), encoding="utf-8")
        asset_id = rows[0]["id"]
        grant = reader.call("POST", f"/api/evidence/media/{asset_id}/playback-grant", {})
        master_url = grant["masterUrl"]
        assert "cloudfront.net" in master_url, grant
        # Query mode: the signed query string goes on every playlist and chunk request (what hls.js does in the UI).
        signed = grant.get("query") or ""

        def with_query(u: str) -> str:
            return u if not signed or "?" in u else f"{u}?{signed.lstrip('?')}"

        status, master = fetch(with_query(master_url))
        assert status == 200 and master.startswith(b"#EXTM3U"), (status, master[:200])
        unsigned = fetch(master_url.split("?")[0])[0]
        assert unsigned in (401, 403), f"CloudFront served the playlist without a signature ({unsigned})"
        base_dir = master_url.split("?")[0].rsplit("/", 1)[0]
        variant_line = next(l for l in master.decode().splitlines() if l and not l.startswith("#"))
        status, variant = fetch(with_query(f"{base_dir}/{variant_line}"))
        assert status == 200 and b"#EXTINF" in variant, (status, variant[:300])
        chunk = next(l for l in variant.decode().splitlines() if l and not l.startswith("#"))
        status, ts = fetch(with_query(f"{base_dir}/{chunk}"))
        assert status == 200 and len(ts) > 1000, (status, len(ts))
        print(f"journal {journal_id}: video plays through CloudFront ({len(master)} B master, rendition {variant_line})")

        Path(ARTIFACTS / "aws-replay-summary.json").write_text(json.dumps({
            "journalId": journal_id, "transportRequestId": tr, "jobId": job, "robot": ROBOT, "assetIds": [r["id"] for r in rows],
            "evidenceDb": os.environ["WARO_E2E_EVIDENCE_DB"], "recordingKeys": side.recording_keys,
        }, indent=1), encoding="utf-8")
    finally:
        if side is not None:
            side._stop.set()
            side.sink.stop()
        if camera is not None:
            camera.stop()
        if sim is not None:
            sim.stop()
        host.stop()
        worker.stop()
        recorder.stop()
