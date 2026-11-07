"""Copy the published map + workflow the harness needs from the dev database into an isolated *test* database.

The dev database is only READ (``SELECT ... jsonb_agg``). Rows are written to the target only, which must be a
database whose name contains "test" (integration tests TRUNCATE it). Re-runnable: rows with the same ids are
replaced. No map asset files are written — Semantic traffic reads the bundle on disk, not the PGM.

    python tools/traffic_e2e/seed_test_db.py --map-code MAP-ONE-V002 --workflow-code INBOUND
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys

CONTAINER = "warotrans-postgres"
USER = "warotrans"


def psql(db: str, sql: str) -> str:
    out = subprocess.run(
        ["docker", "exec", "-i", CONTAINER, "psql", "-U", USER, "-d", db, "-v", "ON_ERROR_STOP=1", "-q", "-t", "-A"],
        input=sql.encode("utf-8"),
        capture_output=True,
        check=False,
    )
    if out.returncode != 0:
        raise RuntimeError(f"psql {db} failed: {out.stderr.decode('utf-8', 'replace')}")
    return out.stdout.decode("utf-8").strip()


def rows(db: str, table: str, where: str) -> list:
    return json.loads(psql(db, f"SELECT coalesce(jsonb_agg(t), '[]'::jsonb) FROM {table} t WHERE {where};") or "[]")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source-db", default="warotrans")
    ap.add_argument("--target-db", default="warotrans_test")
    ap.add_argument("--map-code", default="MAP-ONE-V002")
    ap.add_argument("--workflow-code", default="INBOUND")
    ap.add_argument("--cleanup-e2e-robots", action="store_true",
                    help="only delete the harness's own E2E-* robots (and their commands) from the target, then exit")
    a = ap.parse_args()
    if "test" not in a.target_db.lower():
        print(f"refusing to write to '{a.target_db}': target must be a *test* database", file=sys.stderr)
        return 2
    if a.target_db == a.source_db:
        print("source and target are the same database", file=sys.stderr)
        return 2

    if a.cleanup_e2e_robots:
        # Before the Host starts: robots of an earlier run would otherwise be seeded where they stopped.
        exists = psql(a.target_db, "SELECT to_regclass('fleet.robots') IS NOT NULL;")
        if exists == "t":
            psql(a.target_db, """BEGIN;
DELETE FROM fleet.robot_commands WHERE robot_id IN (SELECT id FROM fleet.robots WHERE code LIKE 'E2E-%');
DELETE FROM fleet.robots WHERE code LIKE 'E2E-%';
COMMIT;""")
        print(json.dumps({"target": a.target_db, "cleanup": "E2E-* robots removed" if exists == "t" else "no schema yet"}))
        return 0

    src = a.source_db
    maps = rows(src, "navigation.map_versions", f"code = '{a.map_code}' AND status = 'PUBLISHED'")
    if len(maps) != 1:
        # The operator moved on to a newer map: the source no longer publishes this one. The harness pins this version
        # (fixtures, bundle, fingerprint), so a copy seeded earlier is exactly what it needs; never touch the source.
        kept = psql(a.target_db, f"SELECT id, warehouse_id FROM navigation.map_versions WHERE code = '{a.map_code}' AND status = 'PUBLISHED';")
        if kept and "|" in kept and "\n" not in kept:
            kept_id, kept_wh = kept.split("|")
            print(json.dumps({"target": a.target_db, "mapVersionId": kept_id, "warehouseId": kept_wh, "reused": True}, indent=2))
            return 0
        raise SystemExit(f"{a.map_code}: expected one PUBLISHED map version in {src}, found {len(maps)}, and none seeded in {a.target_db}")
    mv = maps[0]
    map_id, wh_id = mv["id"], mv["warehouse_id"]
    workflows = rows(src, "execution.workflows", f"code = '{a.workflow_code}' AND status = 'PUBLISHED'")
    if len(workflows) != 1:
        raise SystemExit(f"{a.workflow_code}: expected one PUBLISHED workflow in {src}, found {len(workflows)}")
    wf_id = workflows[0]["id"]

    copy: list[tuple[str, str, list]] = []  # (table, delete-where, rows), parents first
    copy.append(("warehouse.warehouses", f"id = '{wh_id}'", rows(src, "warehouse.warehouses", f"id = '{wh_id}'")))
    copy.append(("navigation.map_versions", f"id = '{map_id}'", maps))
    for t in ("endpoints", "edges", "zones", "endpoint_groups"):
        copy.append((f"navigation.{t}", f"map_version_id = '{map_id}'", rows(src, f"navigation.{t}", f"map_version_id = '{map_id}'")))
    groups = f"endpoint_group_id IN (SELECT id FROM navigation.endpoint_groups WHERE map_version_id = '{map_id}')"
    copy.append(("navigation.endpoint_group_members", groups, rows(src, "navigation.endpoint_group_members", groups)))
    copy.append(("execution.workflows", f"id = '{wf_id}'", workflows))
    tasks_where = f"workflow_id = '{wf_id}'"
    copy.append(("execution.workflow_tasks", tasks_where, rows(src, "execution.workflow_tasks", tasks_where)))
    steps_where = f"workflow_task_id IN (SELECT id FROM execution.workflow_tasks WHERE workflow_id = '{wf_id}')"
    copy.append(("execution.workflow_steps", steps_where, rows(src, "execution.workflow_steps", steps_where)))

    sql = ["BEGIN;"]
    for table, where, data in reversed(copy):  # children first
        sql.append(f"DELETE FROM {table} WHERE {where};")
    for table, _, data in copy:
        payload = json.dumps(data)
        if "$seed$" in payload:
            raise SystemExit("payload contains the SQL quote tag")
        sql.append(f"INSERT INTO {table} SELECT * FROM jsonb_populate_recordset(null::{table}, $seed${payload}$seed$::jsonb);")
    sql.append("COMMIT;")
    psql(a.target_db, "\n".join(sql))
    summary = {table: len(data) for table, _, data in copy}
    print(json.dumps({"target": a.target_db, "mapVersionId": map_id, "warehouseId": wh_id, "workflowId": wf_id,
                      "rows": summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
