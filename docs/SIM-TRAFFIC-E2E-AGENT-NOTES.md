# Sim Traffic Active E2E — agent notes (2026-10-25, unattended)

Operator asked for BE↔FE↔Gazebo Traffic Active prep while away.
**SIMULATION ACCEPTED: NO** — full inbound job COMPLETED was not achieved.
Stack was brought up and multiple real blockers were fixed; remaining issue is FollowPath corridor/collision under Active gate.

## What works now (evidence)

| Layer | Evidence |
|---|---|
| DB route graph V018 | 13 edges / densified GeoJSON 108 edges loaded by `route_server` |
| Host `Traffic__Mode=Active` | `Segmented traffic coordinator started in Active mode` |
| FE | `http://localhost:5173` HTTP 200 |
| Real TransportRequest | `run-job-sim.ps1 -Scenario inbound` creates REQ+JOB, staff confirms PICKUP ×3 |
| Active grant | Host `Active NAVIGATE+authority … edges=3` |
| Robot Active path | `CommandAccepted` NAVIGATE; FollowPath starts (`begin computing control effort`) |
| Host RENEW | `TrafficAuthorityRenewHostedService` + publisher `RENEW_AUTHORITY` every ~1.5s; robot `CommandAccepted` renew |

## Fixes applied this session

### warotrans-system (Host)

1. **`TrafficAuthorityRenewHostedService`** — periodic renew while phase is Authorized/Moving.
2. **`TrafficAuthorityPublisher`** — handles `AuthorityRenewed` → MQTT `RENEW_AUTHORITY`; clears active cmd on revoke/quarantine.
3. **`TrafficState.RenewAuthority`** — bumps seq + emits renew event; **does not renew during HoldingService** (avoids seq race vs NAVIGATE).
4. **`TrafficOptions`** — `AuthorityTtlSec=30`, `AuthorityRenewIntervalSec=1.5`.

### warotrans (robot / sim)

1. FollowPath `controller_id=FollowPath` + edge polylines (full GeoJSON vertices).
2. DynamicEdges soft-continue on spin timeout.
3. Launch clears `/tmp/warotrans_authority_state.json` (stale `lastSeq` → `AUTHORITY_STALE_SEQ`).
4. SafetySupervisor now gets real `pose_age` / `scan_age`; sim overlay ages 2.0s; pause escalate 30s.
5. Helpers: `_agent_launch_sim_active.sh`, `_agent_wait_localized.py`, `_agent_mqtt_sniff.py`.

## Remaining blockers (why job still REASSIGNING)

1. **`RegulatedPurePursuitController detected collision ahead!`** right after FollowPath accept — robot barely moves (~0.03 m), sometimes controller logs `Reached the goal!` without reaching S01.
2. **`cmd_vel_authority_gate` BLOCKING** when `authority_ok` false/stale during pause — motion starved.
3. **Edge ID / path geometry** — Host grants 3 compiled traffic edges; Nav2 uses densified 108-edge GeoJSON. Soft DynamicEdges + sparse FollowPath path may not match a clear corridor on the costmap.
4. Map validation still `isValid=false` (clearance/junction) — warn-only for Active grants.
5. Only workflow `INBOUND_PUTAWAY` in DB (no relocation).

## Stack left for when you return

Prefer a **clean restart** (zombie `gz sim` / SHM lock caused map-frame loss earlier):

```powershell
# 1) Kill leftover sim
wsl -e bash -lc 'pkill -9 -f "gz sim"; pkill -9 -f "ros2 launch"; rm -f /tmp/warotrans_authority_state.json'

# 2) Backend Active
$env:Traffic__Mode='Active'
$env:Mqtt__Host='127.0.0.1'
powershell -File samples/simulation/run-backend-sim.ps1

# 3) Sim (wait until localized)
wsl -e bash /mnt/e/programming/my_project/warotrans/ros2_ws/scripts/_agent_launch_sim_active.sh
wsl -e bash -lc 'PYTHONPATH=/home/ser/pydeps python3 /mnt/e/programming/my_project/warotrans/ros2_ws/scripts/_agent_wait_localized.py 90'

# 4) FE
cd web; npm run dev

# 5) Real TR
powershell -File samples/simulation/run-job-sim.ps1 -Scenario inbound
```

Watch: Gazebo (`gui:=true` if you want window) + Host `Active NAVIGATE+authority` + gateway `CommandAccepted` + **no** continuous `collision ahead` / `authority gate BLOCKING`.

## Suggested next debug (priority)

1. Dump granted `segmentEdgeIds` vs GeoJSON polylines vs robot pose at grant time.
2. Temporarily publish FollowPath path on `/plan` for RViz overlay.
3. Relax local costmap inflation **sim-only** or fix V018 lane clearance so RPP does not see collision on lane centerline.
4. Confirm `authority_ok` true for full segment duration (`ros2 topic echo /traffic/authority_ok`).
