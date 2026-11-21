"""Keeps a Host and an AWS-mode Recorder running on the web dev server's default ports (5090 / 5095) against the AWS
replay E2E databases, so the Technician Replay screens can be looked at with the journal that test produced.

  WARO_E2E_AWS=1 WAROTRANS_SYSTEM_REPO=<system checkout> python tools/traffic_e2e/replay_ui_stack.py
  (then: cd web && npm run dev; sign in as admin / Admin@123 of the *test* database). Ctrl+C stops both.

Never against the developer's databases: warotrans_test and warotrans_evidence_aws_e2e_test only; the Host's MQTT
topic tree is the harness one, so no real robot is ever addressed.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

os.environ.setdefault("WARO_E2E_EVIDENCE_DB", "warotrans_evidence_aws_e2e_test")

from harness import Host  # noqa: E402
from test_evidence_aws_replay_e2e import BUNDLE, AwsRecorder  # noqa: E402

ARTIFACTS = Path(__file__).resolve().parent / "artifacts"


def main() -> None:
    ARTIFACTS.mkdir(exist_ok=True)
    recorder = AwsRecorder(ARTIFACTS, core_base="http://127.0.0.1:5090", port=5095)
    host = Host(ARTIFACTS, port=5090, name="replay-ui", extra_env={
        "Traffic__SemanticMapDirectory": str(BUNDLE),
        "Cors__AllowedOrigins__0": "http://localhost:5173",
    })
    recorder.start()
    try:
        host.start(wait_for_lease=False)
        print("replay UI stack up: Host http://127.0.0.1:5090, Recorder http://127.0.0.1:5095", flush=True)
        while True:
            time.sleep(5)
            if host.proc is None or host.proc.poll() is not None or recorder.proc is None or recorder.proc.poll() is not None:
                raise SystemExit("a process of the stack stopped")
    finally:
        host.stop()
        recorder.stop()


if __name__ == "__main__":
    main()
