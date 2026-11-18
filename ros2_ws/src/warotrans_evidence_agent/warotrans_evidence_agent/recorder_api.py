"""HTTP client of the Evidence Recorder's robot API (urllib only: nothing to install on the Pi).

Authenticates with the robot's recording credential: ``Authorization: Robot <code>.<token>`` (issued once by an admin,
kept in an env file on the robot, never in a repository). Uploads go straight to the presigned URL the Recorder grants
(S3, or the Recorder's local store in development) with the headers it prescribes; no AWS credential lives on the Pi.
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from typing import Any, Optional


class ApiError(Exception):
    def __init__(self, status: int, body: Any, what: str) -> None:
        super().__init__(f"{what} -> {status}: {str(body)[:200]}")
        self.status = status
        self.body = body

    @property
    def code(self) -> Optional[str]:
        return self.body.get("code") if isinstance(self.body, dict) else None

    @property
    def transient(self) -> bool:
        """Worth retrying later (network, server, rate limit, not bound yet)."""
        return self.status in (0, 408, 409, 429) or self.status >= 500


class RecorderApi:
    def __init__(self, base_url: str, robot_code: str, token: str, timeout: float = 15.0) -> None:
        self.base = base_url.rstrip("/")
        self.auth = f"Robot {robot_code}.{token}"
        self.timeout = timeout

    def _call(self, method: str, path: str, body: Any = None, ok=(200, 204)) -> Any:
        data = None if body is None else json.dumps(body).encode("utf-8")
        req = urllib.request.Request(f"{self.base}/api/evidence/edge{path}", data=data, method=method)
        req.add_header("Authorization", self.auth)
        req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw, status = resp.read(), resp.status
        except urllib.error.HTTPError as exc:
            raw, status = exc.read(), exc.code
        except (urllib.error.URLError, OSError) as exc:
            raise ApiError(0, str(exc), f"{method} {path}") from exc
        parsed = json.loads(raw) if raw else {}
        if status not in ok:
            raise ApiError(status, parsed, f"{method} {path}")
        return parsed if status != 204 else None

    # ------------------------------------------------------------------ endpoints

    def whoami(self) -> dict:
        return self._call("GET", "/whoami")

    def active_run(self) -> Optional[dict]:
        return self._call("GET", "/active-run")

    def start_recording(self, body: dict) -> dict:
        return self._call("POST", "/recordings", body)

    def register_segments(self, recording_id: str, segments: list[dict]) -> dict:
        return self._call("POST", f"/recordings/{recording_id}/segments", {"segments": segments})

    def upload_grant(self, segment_id: str) -> dict:
        return self._call("POST", f"/segments/{segment_id}/upload-grant")

    def finalize(self, recording_id: str, body: dict) -> dict:
        return self._call("POST", f"/recordings/{recording_id}/finalize", body)

    def heartbeat(self, recording_id: str, body: dict) -> None:
        self._call("POST", f"/recordings/{recording_id}/heartbeat", body)

    def trajectory(self, run_id: str, points: list[dict]) -> dict:
        return self._call("POST", f"/runs/{run_id}/trajectory", {"points": points})

    def events(self, run_id: str, events: list[dict]) -> dict:
        return self._call("POST", f"/runs/{run_id}/events", {"events": events})

    def plans(self, run_id: str, plans: list[dict]) -> dict:
        return self._call("POST", f"/runs/{run_id}/plans", {"plans": plans})

    def clock_anchors(self, anchors: list[dict]) -> dict:
        return self._call("POST", "/clock-anchors", {"anchors": anchors})


def put_file(grant: dict, path: str, rate_limit_bps: Optional[float] = None, timeout: Optional[float] = None) -> int:
    """PUTs a file to a presigned URL with the granted headers. Optional bandwidth cap in bytes/s (Nav2 keeps the
    network). The timeout follows the size: a 36 MB segment at 2 MB/s needs 18 s, never a fixed 120 s guess."""
    with open(path, "rb") as f:
        body = f.read()
    size = len(body)
    if timeout is None:
        timeout = 60.0 + (size / rate_limit_bps * 1.5 if rate_limit_bps else size / 250_000)
    req = urllib.request.Request(grant["url"], data=_Throttled(body, rate_limit_bps), method=grant.get("method", "PUT"))
    for name, value in (grant.get("headers") or {}).items():
        req.add_header(name, value)
    req.add_header("Content-Length", str(size))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status
    except urllib.error.HTTPError as exc:
        raise ApiError(exc.code, exc.read()[:300].decode(errors="replace"), "PUT segment") from exc
    except (urllib.error.URLError, OSError) as exc:
        raise ApiError(0, str(exc), "PUT segment") from exc


class _Throttled:
    """A bytes body read in 64 KiB pieces, paced to at most ``bps`` bytes per second."""

    def __init__(self, body: bytes, bps: Optional[float]) -> None:
        self.body, self.bps, self.pos = body, bps, 0
        self.t0 = time.monotonic()

    def __len__(self) -> int:
        return len(self.body)

    def __iter__(self):
        while self.pos < len(self.body):
            chunk = self.body[self.pos:self.pos + 65536]
            self.pos += len(chunk)
            if self.bps:
                ahead = self.pos / self.bps - (time.monotonic() - self.t0)
                if ahead > 0:
                    time.sleep(ahead)
            yield chunk
