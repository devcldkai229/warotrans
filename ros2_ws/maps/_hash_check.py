import json
g=json.load(open("/home/ser/maps/route_graph_wh42cfe65f_v003.geojson",encoding="utf-8"))
print("keys", list(g.keys()))
print("name", g.get("name"))
print("properties", g.get("properties"))
print("feature0 props keys", list((g["features"][0].get("properties") or {}).keys())[:12] if g.get("features") else None)
