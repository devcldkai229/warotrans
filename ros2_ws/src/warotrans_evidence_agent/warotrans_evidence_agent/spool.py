"""Durable local queue of everything the robot owes the Evidence Recorder (spec §9.2, §12.1).

SQLite in WAL mode on the robot's SSD: survives a reboot or a power cut, and lets capture go on while the network is
down. Nothing is deleted before the Recorder confirmed it; a full spool stops new recording instead of throwing away
evidence not yet uploaded.

Tables
  recordings  one camera session: local key, the run it was bound to (once the Recorder knew it), state
  segments    closed AVI files: metadata, registered?, uploaded?
  outbox      telemetry batches (trajectory / events / plans / anchors) waiting to be sent, per run
"""

from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS recordings (
    key TEXT PRIMARY KEY,
    started_at_ns INTEGER NOT NULL,
    ended_at_ns INTEGER,
    boot_session_id TEXT NOT NULL,
    job_step_id TEXT,
    directory TEXT NOT NULL,
    recording_id TEXT,
    run_id TEXT,
    state TEXT NOT NULL DEFAULT 'RECORDING',   -- RECORDING, ENDED, FINALIZED
    profile TEXT NOT NULL,
    attempts INTEGER NOT NULL DEFAULT 0,
    next_attempt_at REAL NOT NULL DEFAULT 0,
    last_error TEXT
);
CREATE TABLE IF NOT EXISTS segments (
    segment_id TEXT PRIMARY KEY,
    recording_key TEXT NOT NULL REFERENCES recordings(key),
    sequence_number INTEGER NOT NULL,
    info TEXT NOT NULL,
    registered INTEGER NOT NULL DEFAULT 0,
    uploaded INTEGER NOT NULL DEFAULT 0,
    attempts INTEGER NOT NULL DEFAULT 0,
    next_attempt_at REAL NOT NULL DEFAULT 0,
    last_error TEXT,
    UNIQUE (recording_key, sequence_number)
);
CREATE TABLE IF NOT EXISTS outbox (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    kind TEXT NOT NULL,          -- trajectory, events, plans, anchors
    run_id TEXT,                 -- null for anchors
    payload TEXT NOT NULL,
    attempts INTEGER NOT NULL DEFAULT 0,
    next_attempt_at REAL NOT NULL DEFAULT 0,
    created_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS kv (k TEXT PRIMARY KEY, v TEXT NOT NULL);
"""


@dataclass
class Recording:
    key: str
    started_at_ns: int
    ended_at_ns: Optional[int]
    boot_session_id: str
    job_step_id: Optional[str]
    directory: str
    recording_id: Optional[str]
    run_id: Optional[str]
    state: str
    profile: dict
    attempts: int


class Spool:
    def __init__(self, path: str, clock=time.time) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._db = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=FULL")
        self._db.executescript(SCHEMA)
        self._lock = threading.Lock()
        self._clock = clock

    def close(self) -> None:
        self._db.close()

    def _q(self, sql: str, args: tuple = ()) -> list[tuple]:
        with self._lock:
            return self._db.execute(sql, args).fetchall()

    # ------------------------------------------------------------------ recordings

    def add_recording(self, key: str, started_at_ns: int, boot_session_id: str, job_step_id: Optional[str],
                      directory: str, profile: dict) -> None:
        self._q("INSERT OR IGNORE INTO recordings(key, started_at_ns, boot_session_id, job_step_id, directory, profile)"
                " VALUES (?,?,?,?,?,?)", (key, started_at_ns, boot_session_id, job_step_id, directory, json.dumps(profile)))

    def end_recording(self, key: str, ended_at_ns: int) -> None:
        self._q("UPDATE recordings SET ended_at_ns=?, state='ENDED' WHERE key=? AND state='RECORDING'", (ended_at_ns, key))

    def bind_recording(self, key: str, recording_id: str, run_id: str) -> None:
        self._q("UPDATE recordings SET recording_id=?, run_id=?, attempts=0, next_attempt_at=0, last_error=NULL WHERE key=?",
                (recording_id, run_id, key))

    def finalize_recording(self, key: str) -> None:
        self._q("UPDATE recordings SET state='FINALIZED' WHERE key=?", (key,))

    def recording_failed(self, key: str, error: str, backoff_s: float) -> None:
        self._q("UPDATE recordings SET attempts=attempts+1, next_attempt_at=?, last_error=? WHERE key=?",
                (self._clock() + backoff_s, error[:300], key))

    def recordings(self, states: tuple[str, ...] = ("RECORDING", "ENDED"), due_only: bool = False) -> list[Recording]:
        marks = ",".join("?" * len(states))
        sql = f"SELECT key, started_at_ns, ended_at_ns, boot_session_id, job_step_id, directory, recording_id, run_id, state, profile, attempts FROM recordings WHERE state IN ({marks})"
        args: tuple = tuple(states)
        if due_only:
            sql += " AND next_attempt_at <= ?"
            args += (self._clock(),)
        return [Recording(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], json.loads(r[9]), r[10])
                for r in self._q(sql + " ORDER BY started_at_ns", args)]

    def recording(self, key: str) -> Optional[Recording]:
        rows = [r for r in self.recordings(("RECORDING", "ENDED", "FINALIZED")) if r.key == key]
        return rows[0] if rows else None

    # ------------------------------------------------------------------ segments

    def add_segment(self, info: dict) -> bool:
        """True when new (the same segment reported twice is stored once)."""
        with self._lock:
            cur = self._db.execute(
                "INSERT OR IGNORE INTO segments(segment_id, recording_key, sequence_number, info) VALUES (?,?,?,?)",
                (info["segment_id"], info["recording_key"], int(info["sequence_number"]), json.dumps(info)))
            return cur.rowcount == 1

    def segments(self, recording_key: str) -> list[dict]:
        rows = self._q("SELECT info, registered, uploaded, attempts FROM segments WHERE recording_key=? ORDER BY sequence_number",
                       (recording_key,))
        return [dict(json.loads(r[0]), registered=bool(r[1]), uploaded=bool(r[2]), attempts=r[3]) for r in rows]

    def mark_registered(self, segment_ids: list[str]) -> None:
        for sid in segment_ids:
            self._q("UPDATE segments SET registered=1 WHERE segment_id=?", (sid,))

    def mark_uploaded(self, segment_id: str) -> None:
        self._q("UPDATE segments SET uploaded=1, last_error=NULL WHERE segment_id=?", (segment_id,))

    def segment_failed(self, segment_id: str, error: str, backoff_s: float) -> None:
        self._q("UPDATE segments SET attempts=attempts+1, next_attempt_at=?, last_error=? WHERE segment_id=?",
                (self._clock() + backoff_s, error[:300], segment_id))

    def due_uploads(self, recording_key: str) -> list[dict]:
        rows = self._q("SELECT info FROM segments WHERE recording_key=? AND registered=1 AND uploaded=0 AND next_attempt_at<=?"
                       " ORDER BY sequence_number", (recording_key, self._clock()))
        return [json.loads(r[0]) for r in rows]

    def pending_upload_count(self) -> int:
        return self._q("SELECT COUNT(*) FROM segments WHERE uploaded=0")[0][0]

    def uploaded_files(self) -> list[str]:
        """Files of finalized recordings whose segments are all confirmed: the only ones that may be deleted."""
        rows = self._q("SELECT s.info FROM segments s JOIN recordings r ON r.key = s.recording_key"
                       " WHERE r.state='FINALIZED' AND s.uploaded=1")
        out = []
        for (info,) in rows:
            data = json.loads(info)
            out += [data["path"], data.get("sidecar_path", "")]
        return [p for p in out if p]

    # ------------------------------------------------------------------ telemetry outbox

    def enqueue(self, kind: str, run_id: Optional[str], payload: Any) -> None:
        self._q("INSERT INTO outbox(kind, run_id, payload, created_at) VALUES (?,?,?,?)",
                (kind, run_id, json.dumps(payload), self._clock()))

    def due_batches(self, limit: int = 20) -> list[tuple[int, str, Optional[str], Any]]:
        rows = self._q("SELECT id, kind, run_id, payload FROM outbox WHERE next_attempt_at<=? ORDER BY id LIMIT ?",
                       (self._clock(), limit))
        return [(r[0], r[1], r[2], json.loads(r[3])) for r in rows]

    def batch_done(self, batch_id: int) -> None:
        self._q("DELETE FROM outbox WHERE id=?", (batch_id,))

    def batch_failed(self, batch_id: int, backoff_s: float) -> None:
        self._q("UPDATE outbox SET attempts=attempts+1, next_attempt_at=? WHERE id=?", (self._clock() + backoff_s, batch_id))

    def outbox_size(self) -> int:
        return self._q("SELECT COUNT(*) FROM outbox")[0][0]

    # ------------------------------------------------------------------ counters

    def next_value(self, name: str) -> int:
        """A counter that survives restarts (trajectory sequence, plan version)."""
        with self._lock:
            row = self._db.execute("SELECT v FROM kv WHERE k=?", (name,)).fetchone()
            value = int(row[0]) + 1 if row else 1
            self._db.execute("INSERT OR REPLACE INTO kv(k, v) VALUES (?,?)", (name, str(value)))
            return value
