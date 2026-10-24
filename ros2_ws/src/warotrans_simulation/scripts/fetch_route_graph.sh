#!/usr/bin/env bash
# Fetch the Nav2 Route Server lane graph of a map version from the WaroTrans backend.
#
#   fetch_route_graph.sh <backend_url> <map_version_id> <out_path> [--max-speed M] [--user U --password P]
#   fetch_route_graph.sh http://localhost:5090 72bffbce-... ~/maps/route_graph.geojson
#
# Auth: WAROTRANS_TOKEN (Bearer token) if set, otherwise login with --user / --password
# (default admin / Admin@123 = the dev seed; never use it against a real deployment).
#
# Speed limits: the backend writes abs_speed_limit (m/s) on lane edges. nav2_route 1.3.13 (Jazzy) AdjustSpeedLimit
# only reads "speed_limit" as a PERCENTAGE of the controller's desired_linear_vel, so this script adds
# speed_limit = min(100, abs_speed_limit / max_speed * 100). --max-speed must be that desired_linear_vel:
# 0.35 in simulation (nav2_params_sim.yaml, default here), 0.10 on the real robot (nav2_params.yaml).
#
# Exit: 0 graph written, 2 the map's lanes do not form a usable graph (422: issues printed), 1 other errors.
set -euo pipefail

usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
[ $# -ge 3 ] || usage
BASE_URL="${1%/}"; MAP_ID="$2"; OUT="$3"; shift 3
MAX_SPEED=0.35; USER_NAME=admin; PASSWORD='Admin@123'
while [ $# -gt 0 ]; do
  case "$1" in
    --max-speed) MAX_SPEED="$2"; shift 2 ;;
    --user) USER_NAME="$2"; shift 2 ;;
    --password) PASSWORD="$2"; shift 2 ;;
    *) usage ;;
  esac
done

TOKEN="${WAROTRANS_TOKEN:-}"
if [ -z "$TOKEN" ]; then
  LOGIN=$(curl -sS -X POST "$BASE_URL/api/identity/login" -H "Content-Type: application/json" \
    -d "{\"username\":\"$USER_NAME\",\"password\":\"$PASSWORD\"}")
  TOKEN=$(printf '%s' "$LOGIN" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("accessToken",""))' 2>/dev/null || true)
  [ -n "$TOKEN" ] || { echo "login failed at $BASE_URL: $LOGIN" >&2; exit 1; }
fi

BODY=$(mktemp); trap 'rm -f "$BODY"' EXIT
STATUS=$(curl -sS -o "$BODY" -w '%{http_code}' -H "Authorization: Bearer $TOKEN" \
  "$BASE_URL/api/navigation/maps/$MAP_ID/route-graph")

case "$STATUS" in
  200)
    mkdir -p "$(dirname "$OUT")"
    python3 - "$BODY" "$OUT" "$MAX_SPEED" <<'PY'
import json, sys
src, out, max_speed = sys.argv[1], sys.argv[2], float(sys.argv[3])
graph = json.load(open(src, encoding="utf-8"))
nodes = edges = limited = 0
for feature in graph["features"]:
    if feature["geometry"]["type"] == "Point":
        nodes += 1
        continue
    edges += 1
    metadata = feature["properties"].setdefault("metadata", {})
    if "abs_speed_limit" in metadata:
        metadata["speed_limit"] = round(min(100.0, float(metadata["abs_speed_limit"]) / max_speed * 100.0), 1)
        limited += 1
json.dump(graph, open(out, "w", encoding="utf-8"), indent=2)
print(f"route graph {graph.get('name')}: {nodes} nodes, {edges} edges, {limited} speed-limited "
      f"(% of {max_speed} m/s) -> {out}")
PY
    ;;
  422)
    echo "The map's lanes do not form a usable route graph (HTTP 422). Fix them in the web Map Editor:" >&2
    python3 - "$BODY" <<'PY' >&2
import json, sys
problem = json.load(open(sys.argv[1], encoding="utf-8"))
for issue in problem.get("issues", []):
    print(f"  {issue['severity']:<7} {issue['code']:<22} {issue['message']}")
PY
    exit 2 ;;
  *)
    echo "GET route-graph failed: HTTP $STATUS" >&2; cat "$BODY" >&2; echo >&2; exit 1 ;;
esac
