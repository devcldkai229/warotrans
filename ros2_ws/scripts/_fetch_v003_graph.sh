#!/usr/bin/env bash
set -euo pipefail
OUT=/home/ser/maps/route_graph_wh42cfe65f_v003.geojson
MAP=1143c8a3-ae62-4660-b684-bc462450f30f
BASE=http://127.0.0.1:5090
LOGIN=$(curl -sS -X POST "$BASE/api/identity/login" -H 'Content-Type: application/json' \
  -d '{"username":"admin","password":"Admin@123"}')
TOKEN=$(printf '%s' "$LOGIN" | python3 -c 'import json,sys; print(json.load(sys.stdin)["accessToken"])')
curl -sS -D /tmp/rg.hdr -H "Authorization: Bearer $TOKEN" \
  "$BASE/api/navigation/maps/$MAP/route-graph" -o "$OUT"
echo "header:" $(grep -i x-route-graph-hash /tmp/rg.hdr || true)
python3 - <<'PY'
import json
g=json.load(open("/home/ser/maps/route_graph_wh42cfe65f_v003.geojson",encoding="utf-8"))
print("name", g.get("name"))
print("graphHash", (g.get("properties") or {}).get("graphHash"))
print("features", len(g.get("features") or []))
PY
cp "$OUT" /mnt/e/programming/my_project/warotrans/ros2_ws/maps/route_graph_wh42cfe65f_v003.geojson
cp "$OUT" /mnt/e/programming/my_project/warotrans-system/samples/maps/MAP-WH42CFE65F-ONEWAY/route-graph.geojson
