# RouteGraph retirement report (Stage 3)

## Verdict

**Semantic Active runtime no longer requires RouteGraph compile, BindGraph, edge A*, Nav2 Route Server, or `graphHash` path identity.**  
Offline Check / Debug SVG / GeoJSON export **kept**. Source files **not deleted** (rollback via git).

## RETIRE from Active Semantic path

| Dependency | Status |
|---|---|
| `JobExecutionEngine` compile → `RequestRoute` / `AcquireRoute` when `TrafficCoordinatorMode.Semantic` | Unlinked (Semantic branch issues Navigate only) |
| Robot `require_graph_hash` / RouteEnforcer for `semantic_active` | Unlinked (`route_enforcer=None`) |
| Nav2 `route_server` on MAP-ONE dual/semantic launch | Empty `route_graph` |
| Map Editor “must Check RouteGraph before Active” UX | Demoted to offline/legacy labels |

## KEEP (offline / QA)

- `RouteGraphBuilder`, `TrafficGraphCompiler`, `GetRouteGraph`, validation, debug SVG
- `TrafficCoordinatorMode.Active` edge-grant path (legacy) until operators migrate configs to `Semantic`
- JUNCTION zone type in editor (optional geometry)

## REPLACE identity

| Old | New (Semantic) |
|---|---|
| `graphHash` + `segmentEdgeIds` | `semanticMapFingerprint` + `planRevision` + `resourceIds` (`zone:` / `cell:`) |
| Progress `EdgeId` | Cleared `resourceIds` + pose clearance |

## Rollback

1. Set `Traffic:Mode=Active` and restore Active gateway overlays with `route_graph`.
2. Revert Semantic JobExecutionEngine branch / `semantic_active` gateway if needed.
3. Do **not** need to restore deleted files — none were removed in Stage 3.
