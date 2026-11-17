"""Backend process + REST helpers for the traffic E2E harness (isolated DB, isolated MQTT topic tree)."""

from __future__ import annotations

import json
import os
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Optional

ROBOT_REPO = Path(__file__).resolve().parents[2]
SYSTEM_REPO = Path(os.environ.get("WAROTRANS_SYSTEM_REPO", ROBOT_REPO.parent / "warotrans-system"))
HOST_DLL = SYSTEM_REPO / "backend" / "src" / "WaroTrans.Host" / "bin" / "Debug" / "net10.0" / "WaroTrans.Host.dll"
BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / "map_one_v004"
RECORDER_DLL = (SYSTEM_REPO / "backend" / "src" / "WaroTrans.Evidence" / "Recorder" / "bin" / "Debug" / "net10.0"
                / "WaroTrans.Evidence.Recorder.dll")

# Evidence Recorder of the harness (docs/adr/0012 of warotrans-system): own port, own *test* database, own token.
EVIDENCE_DB = os.environ.get("WARO_E2E_EVIDENCE_DB", "warotrans_evidence_e2e_test")
EVIDENCE_PORT = int(os.environ.get("WARO_E2E_EVIDENCE_PORT", "5198"))
EVIDENCE_TOKEN = "warotrans-e2e-evidence-service-token-0123456789"

TEST_DB = os.environ.get("WARO_E2E_DB", "warotrans_test")
PREFIX = os.environ.get("WARO_E2E_PREFIX", "warotrans-e2e/v1")
PORT = int(os.environ.get("WARO_E2E_PORT", "5199"))
BASE = f"http://127.0.0.1:{PORT}"

if "test" not in TEST_DB.lower():  # never TRUNCATE-able dev data
    raise SystemExit(f"WARO_E2E_DB={TEST_DB}: the harness only runs against a *test* database")
if "test" not in EVIDENCE_DB.lower():
    raise SystemExit(f"WARO_E2E_EVIDENCE_DB={EVIDENCE_DB}: the harness only runs against a *test* database")
if PREFIX.strip("/") == "warotrans/v1":
    raise SystemExit("the harness must not use the live topic prefix warotrans/v1 (real robots listen there)")


class Host:
    """The real WaroTrans.Host (current build) in Semantic mode against the test database."""

    def __init__(self, log_dir: Path, port: int = PORT, name: str = "host", extra_env: Optional[dict] = None) -> None:
        self.log_dir = log_dir
        self.extra_env = extra_env or {}
        self.port = port
        self.name = name
        self.proc: Optional[subprocess.Popen] = None
        self.starts = 0

    @property
    def base(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def env(self) -> dict[str, str]:
        e = dict(os.environ)
        e.update({
            "ASPNETCORE_ENVIRONMENT": "Development",
            "ASPNETCORE_URLS": self.base,
            "ConnectionStrings__PostgreSQL": f"Host=127.0.0.1;Port=5433;Database={TEST_DB};Username=warotrans;Password=warotrans",
            "ConnectionStrings__MongoDB": "mongodb://warotrans:warotrans@localhost:27018/?authSource=admin",
            "Mongo__ConnectionString": "mongodb://warotrans:warotrans@localhost:27018/?authSource=admin",
            "Mongo__DatabaseName": "warotrans_operations_test",
            "MapAssetStorage__Provider": "Local",
            "Mqtt__Host": "127.0.0.1",
            "Mqtt__Port": "1883",
            "Mqtt__TopicPrefix": PREFIX,
            "Mqtt__ClientId": f"warotrans-e2e-{self.name}",
            "Mqtt__HeartbeatTimeoutSeconds": "5",
            "Mqtt__CommandAckTimeoutSeconds": "15",
            "Traffic__Mode": "Semantic",
            "Traffic__SemanticMapDirectory": str(BUNDLE),
            "Traffic__SemanticHorizonMeters": "4.0",
            "Traffic__SemanticCellSizeMeters": "0.25",
            "Traffic__SemanticPathToleranceMeters": "0.15",
            "Traffic__RequireCoordinatorLease": "true",
            "Traffic__AuthorityRenewIntervalSec": "1.0",
            "Traffic__AuthorityTtlSec": "30",
            "Logging__Console__FormatterName": "simple",
            "Logging__Console__FormatterOptions__TimestampFormat": "HH:mm:ss.fff ",
            "Logging__LogLevel__Default": "Information",
            "Logging__LogLevel__Microsoft.EntityFrameworkCore": "Warning",
            # Never push test events to a developer's Recorder on :5095; EvidenceRecorder.host_env() turns it on.
            "EvidenceBridge__Enabled": "false",
        })
        e.update(self.extra_env)
        return e

    def start(self, wait_for_lease: bool = True) -> None:
        try:
            self._start(wait_for_lease)
        except BaseException:
            # A Host that came up without the lease (or never answered) keeps the port and the advisory lock of
            # warotrans_test: left running, it makes every later run fail the same way. Stop it before failing.
            self.stop()
            raise

    def _start(self, wait_for_lease: bool) -> None:
        if not HOST_DLL.exists():
            raise SystemExit(f"{HOST_DLL} missing: dotnet build backend/src/WaroTrans.Host first")
        self.starts += 1
        out = open(self.log_dir / f"{self.name}-{self.starts}.log", "wb")
        self.proc = subprocess.Popen(["dotnet", str(HOST_DLL)], cwd=HOST_DLL.parent, env=self.env(),
                                     stdout=out, stderr=subprocess.STDOUT)
        deadline = time.time() + 90
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(f"Host exited ({self.proc.returncode}); see {out.name}")
            try:
                urllib.request.urlopen(f"{self.base}/api/fleet/ping", timeout=2).read()
                break
            except (urllib.error.URLError, OSError):
                time.sleep(0.5)
        else:
            raise RuntimeError("Host did not come up within 90 s")
        if not wait_for_lease:
            return
        # Grants open only once the lease is held and the epoch announced.
        deadline = time.time() + 30
        while time.time() < deadline:
            if "grants open" in self.log_text():
                return
            time.sleep(0.3)
        # The event log line is internal; fall back to the info log of the lease service.
        if "grants open" not in self.log_text():
            raise RuntimeError("traffic coordinator never opened for grants")

    def log_text(self) -> str:
        return (self.log_dir / f"{self.name}-{self.starts}.log").read_text(encoding="utf-8", errors="replace")

    def stop(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait(timeout=20)
        self.proc = None


class EvidenceRecorder:
    """The real Evidence Recorder (current build) on its own test database, fed by the harness Host's EvidenceBridge."""

    def __init__(self, log_dir: Path, core_base: str = BASE, port: int = EVIDENCE_PORT) -> None:
        self.log_dir = log_dir
        self.core_base = core_base
        self.port = port
        self.proc: Optional[subprocess.Popen] = None

    @property
    def base(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    @staticmethod
    def available() -> bool:
        return RECORDER_DLL.exists()

    def env(self) -> dict[str, str]:
        e = dict(os.environ)
        e.update({
            "ASPNETCORE_ENVIRONMENT": "Development",
            "ASPNETCORE_URLS": self.base,
            "ConnectionStrings__Evidence": f"Host=127.0.0.1;Port=5433;Database={EVIDENCE_DB};Username=warotrans;Password=warotrans",
            "Evidence__ServiceToken": EVIDENCE_TOKEN,
            "Evidence__CoreBaseUrl": self.core_base,
            "Evidence__ReconcileIntervalSeconds": "30",
            "Evidence__MaintenanceIntervalSeconds": "10",
            "Logging__Console__FormatterName": "simple",
            "Logging__Console__FormatterOptions__TimestampFormat": "HH:mm:ss.fff ",
        })
        return e

    def host_env(self) -> dict[str, str]:
        """What the harness Host needs to push its events here."""
        return {
            "EvidenceBridge__Enabled": "true",
            "EvidenceBridge__RecorderBaseUrl": self.base,
            "EvidenceBridge__ServiceToken": EVIDENCE_TOKEN,
            "EvidenceBridge__PollIntervalMs": "300",
        }

    def start(self) -> None:
        if not RECORDER_DLL.exists():
            raise SystemExit(f"{RECORDER_DLL} missing: dotnet build backend/src/WaroTrans.Evidence/Recorder first")
        out = open(self.log_dir / "evidence-recorder.log", "wb")
        self.proc = subprocess.Popen(["dotnet", str(RECORDER_DLL)], cwd=RECORDER_DLL.parent, env=self.env(),
                                     stdout=out, stderr=subprocess.STDOUT)
        deadline = time.time() + 60
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(f"Evidence Recorder exited ({self.proc.returncode}); see {out.name}")
            try:
                urllib.request.urlopen(f"{self.base}/health", timeout=2).read()
                return
            except (urllib.error.URLError, OSError):
                time.sleep(0.5)
        self.stop()
        raise RuntimeError("Evidence Recorder did not come up within 60 s")

    def stop(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait(timeout=20)
        self.proc = None

    def journal(self, api: "Api", transport_request_id: str) -> Optional[dict]:
        """The journal of a request as an Admin sees it (with the Host's access token), or None while there is none."""
        reader = Api(self.base)
        reader.token = api.token
        body = reader.call("GET", f"/api/evidence/journals/by-transport-request/{transport_request_id}", ok=(200, 404))
        return body if body.get("id") else None


class Api:
    def __init__(self, base: str = BASE) -> None:
        self.base = base
        self.token = ""

    def login(self, username: str = "admin", password: str = "Admin@123") -> None:
        # Development seeder account of the *test* database (never a real credential).
        body = self.call("POST", "/api/identity/login", {"username": username, "password": password}, auth=False)
        self.token = body.get("accessToken") or body.get("data", {}).get("accessToken")

    def call(self, method: str, path: str, body: Any = None, *, auth: bool = True, ok=(200, 201, 202, 204)) -> Any:
        data = None if body is None else json.dumps(body).encode("utf-8")
        req = urllib.request.Request(f"{self.base}{path}", data=data, method=method)
        req.add_header("Content-Type", "application/json")
        if auth and self.token:
            req.add_header("Authorization", f"Bearer {self.token}")
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                raw = resp.read()
                status = resp.status
        except urllib.error.HTTPError as exc:
            raw, status = exc.read(), exc.code
        parsed = json.loads(raw) if raw else {}
        if status == 401 and auth and self.token and not getattr(self, "_relogging", False):
            # The access token outlives no long scenario (15 min): log in again once and repeat the call.
            self._relogging = True
            try:
                self.login()
            finally:
                self._relogging = False
            return self.call(method, path, body, auth=auth, ok=ok)
        if status not in ok:
            raise AssertionError(f"{method} {path} → {status}: {parsed}")
        return parsed

    def snapshot(self) -> dict[str, Any]:
        return self.call("GET", "/api/fleet/traffic/snapshot")


def wait_for(predicate, timeout: float, interval: float = 0.2, what: str = "condition"):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = predicate()
        if last:
            return last
        time.sleep(interval)
    raise AssertionError(f"timed out after {timeout}s waiting for {what} (last={last!r})")
