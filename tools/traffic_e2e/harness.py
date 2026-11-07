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
BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / "map_one_v002"

TEST_DB = os.environ.get("WARO_E2E_DB", "warotrans_test")
PREFIX = os.environ.get("WARO_E2E_PREFIX", "warotrans-e2e/v1")
PORT = int(os.environ.get("WARO_E2E_PORT", "5199"))
BASE = f"http://127.0.0.1:{PORT}"

if "test" not in TEST_DB.lower():  # never TRUNCATE-able dev data
    raise SystemExit(f"WARO_E2E_DB={TEST_DB}: the harness only runs against a *test* database")
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
        })
        e.update(self.extra_env)
        return e

    def start(self, wait_for_lease: bool = True) -> None:
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
