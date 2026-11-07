# Move the real robot to a newly published map

Semantic traffic ties everything to one bundle: the backend (`Traffic:SemanticMapDirectory`) and the robot gateway
(`semantic_manifest`) must load the same `semantic_manifest.json`. Do these in order; `<MAP>` is e.g. `MAP-ONE-V003`,
`<slug>` e.g. `map_one_v003`. Steps 1-2 only read the database.

1. Publish the map on the web, then export a snapshot (from `warotrans-system`):
   `powershell -ExecutionPolicy Bypass -File samples/maps/export-map-snapshot.ps1 -MapCode <MAP>`
2. Build the bundle (from `warotrans`, WSL or Pi; it refuses a map that is not PUBLISHED):
   `python3 ros2_ws/src/warotrans_navigation/scripts/export_semantic_bundle.py --snapshot ../warotrans-system/samples/maps/<MAP> --out-slug <slug>`
3. Backend: in `Properties/launchSettings.json` (profile `RealRobot`) point `Traffic__SemanticMapDirectory` at the new
   bundle, restart the Host, and check its log: `SemanticPolicyBootstrap bound fingerprint=...`.
4. Deploy: `./tools/deploy.sh --pi waro@warotrans.local --no-delete`, then set the Pi clock:
   `ssh -t waro@warotrans.local "sudo date -u -s @$(date +%s)"`.
5. Move the robot record (Admin; refused while it has a command):
   `PUT /api/fleet/robots/{id}/map` with `{"mapVersionId":"<id from semantic_manifest.json>"}`.
6. On the Pi: `~/warotrans_tools/pi_run_semantic.sh stop`, then
   `BUNDLE=$HOME/ros2_ws/src/warotrans_navigation/maps/<slug> BROKER=<laptop IP> ~/warotrans_tools/pi_run_semantic.sh start`
   (motors stay disabled unless `MOTORS=true`).
7. Read the new EP-PARKING-A pose from the snapshot (`endpoints` in `snapshot.json`; it may have moved), place the
   robot there, then: `python3 ~/warotrans_tools/set_initial_pose.py --x <x> --y <y> --yaw <yaw> --confirm-placed`.
8. `python3 ~/warotrans_tools/real_robot_readiness.py --manifest <bundle>/semantic_manifest.json --mqtt-host <laptop IP> --backend-url http://<laptop IP>:5090 --home-x <x> --home-y <y> --expect-motors-disabled`.
   The fingerprint line must match the backend log. Everything but the E-stop line must PASS.
9. Optional, recommended: regenerate real paths on the Pi with `tools/traffic_e2e/generate_nav2_paths.py`, then rerun
   `test_real_map_geometry.py` (update its bundle path) to see routes whose footprint touches walls.

The endpoint codes the backend needs: `Traffic__RobotHomeEndpoints__RBT-001` must name an endpoint that exists on the
new map, or the first Job is refused with `robot_home_not_on_map`.
