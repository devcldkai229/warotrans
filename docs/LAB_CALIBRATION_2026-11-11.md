# Lab calibration 2026-11-11: RBT-001 on MAP-ONE-V006 (INBOUND area)

The operator placed the robot near EP-INBOUND and authorised supervised, low-speed motion. There was no gateway and no backend: the scripts commanded the robot directly, either through `/cmd_vel_nav` → velocity_smoother → collision_monitor, or through Nav2 `navigate_to_pose`. Every move checked clearance before starting and aborted on stale data or a near obstacle. Scripts and raw logs are on the Pi in `~/wt_calib/` and `~/wt_logs/calib_20261111/`: `motion.jsonl`, `nav.jsonl`, `rec_*.json` (scan, wheel_ticks, odom, cmd at full rate).

**Ground truth.** Each live `/scan` is matched against the V006 occupancy map, independent of odometry and AMCL. The cost is the distance to the wall faces, truncated, so returns from unmapped objects do not pull the result.
- Standing still, the match repeats to under 2 mm and 0.2°, with residual about 0.019 m and 0.7–0.8 inliers.
- A plot confirmed it: the matched scan lies on the map walls, while the AMCL pose at the same instant was visibly rotated (see the AMCL section below).

## Findings

### 1. Open-loop wheel speed is about 2× the command and asymmetric (fixed)
At a 0.3 rad/s in-place turn the target is 0.068 m/s per wheel.
- **Open loop.** The wheels actually ran at 0.13–0.15 m/s, from `min_duty` 35 plus the commissioning map, so the robot turned at about 0.7 rad/s.
- **Asymmetry.** The forward wheel travelled 17–44 % further than the reverse wheel, so the robot crept forward while turning.
- **Closed loop.** The firmware PI (`closed_loop`, `kp` 600, `ki` 1500) held 0.068 ± 0.003 m/s within about 1 s, symmetric, with no oscillation. A straight move held 0.040 m/s exactly. With `kp` 250 / `ki` 800 the speed was still 0.08 m/s after 1 s.

**Now in `hardware.yaml`.** Nav2 legs with it:
- 6 succeeded with 0 recoveries.
- The timeouts happened only with the shifted-frame trial in item 2.

### 2. The robot rotates about a point 12 cm behind `base_footprint` (not changed yet)
Over 8 in-place turns (±45°, ±90°, +180°, open and closed loop), the fixed point of the motion in the robot frame was x = −0.10 … −0.16 m (mean about −0.12) and y ≈ +0.02 m.
- **Consequence.** Every in-place turn moves `base_footprint`, the frame Nav2, AMCL and the wheel odometry all treat as the rotation centre, along an arc. The shift is 0.11 m per 45° and 0.14–0.22 m per 90°, and odometry does not see it.
- **Field symptoms explained:** cutting into the neighbouring lane when turning, "spinning" and re-approaching at endpoints, and stopping off the endpoint.

**Trial.** `base_footprint_to_base_link.x = 0.12` (geometry.yaml) with the Nav2 footprint shifted forward by 0.12.
- The base moved only 0.04 m in a 90° turn, and odom/true went from 0.86 to 0.96.
- It also showed what still has to be done before this can be committed:
  - **Goal offset.** Endpoints mean the chassis centre, so goals to `base_footprint` must be moved back 0.12 m along the heading (gateway).
  - **QUEUE-POINT clearance.** The robot's real swept radius is 0.335 m, not 0.21 m. At EP-QUEUE-POINT it cannot turn to −90° without the nose reaching the shelf block 0.25 m away. The endpoint, or the lane mask grown for planning, has to be reviewed with the real geometry.
  - **Rig and lane-intrusion check** with the shifted footprint.

**Reverted on the Pi** until those are done.

### 3. Rotation slip depends on direction; wheel_separation 0.4535 is right on average
Pairs of opposite turns gave the following odometry / true ratios:

| Pair | CCW | CW |
|---|---|---|
| Open loop | 0.86–0.87 | 1.10–1.32 |
| Closed loop | 0.92–0.96 | 1.12 |

- **On average** the ratio is about 1.0, so the effective separation stays 0.4535.
- **Per turn** the heading error is ±5–30 %. One separation value cannot remove it; a yaw sensor can.
- **IMU.** The BNO055 did not answer on I2C bus 1 (no device at any address, `[Errno 121] Remote I/O error`). Check its wiring and power before EKF can be considered.

### 4. AMCL does not keep up with the heading while the robot moves
With the default wheel-only AMCL (`alpha1` 0.3), after in-place turns and after Nav2 arrivals, AMCL was off the true pose by:

| When | Position | Heading |
|---|---|---|
| After in-place turns | 0.07–0.25 m | 5–18° |
| After Nav2 arrivals | 0.07–0.21 m | 5–24° |

Nav2 reports success against that pose with 0.05 m / 0.08 rad.

The truth of the arrival, in base_link versus the endpoint, over 6 legs:

| Frame | Position error | Heading error |
|---|---|---|
| Old frame | 0.20–0.35 m | 24–66° |
| Shifted-frame trial | 0.15–0.27 m | 6–63° |

- **At rest.** After 3 × `/request_nomotion_update` AMCL moves most of the way back toward the truth. This is the "jump after the stop" seen on 2026-11-10.
- **`alpha1` 0.8.** AMCL yaw error after a 90° turn fell to 1.1° (0.4° after nomotion), but the Pi load average reached 5.8 (2000 particles) and Wi-Fi lagged. Reverted; this needs a lower particle count or a lighter laser model before it can be used.
- **Arrival refine** in the gateway re-sends the goal after nomotion updates and is the existing mitigation. One emulated refine run timed out (7 recoveries) while the Pi was overloaded, so it has no valid result yet.

### 5. Lidar mount yaw is about +1.6°, odometry distance about 1–3 %
- **Lidar.** On straight moves the scan-based heading minus the true direction of motion was −1.55°, −1.69° and −0.89°. That points to a lidar yaw about 1–1.6° larger than the URDF value 1.5708. It is small and was not changed; confirm it with longer straight runs.
- **Distance.** Odometry / true distance: 1.034, 0.988, 1.004.

### 6. Standing still
- Odometry drift was 0 over 180 s.
- AMCL was steady, error 3 mm / 0.1° after an initial pose set from the scan match.
- Scan-match spread was under 8 mm.

## State left on the robot
- `hardware.yaml`: closed loop on, `kp` 600, `ki` 1500 (the only change kept).
- geometry.yaml, nav2_params.yaml and amcl_wheel_only.yaml: restored from `~/wt_calib/*.orig`.
- Backup before the session: `~/warotrans_backups/ros2_ws_src_20261111_115538_precalib.tgz`.
- The stack was restarted with `MOTORS=false`.
- `pi_run_semantic.sh stop` leaves `battery_monitor_node` and `recovery_guard` running. They were killed by hand; the stop pattern should include them.

## Next steps, in order of effect
1. **Rotation centre:** shift `base_footprint` 0.12 m back, with the gateway goal offset, footprint, keepout-planning growth and endpoints reviewed. Then run the rig and a supervised run.
2. **BNO055:** fix the I2C connection, run the turn test, and enable EKF if it holds heading within 2 % over 360°.
3. **AMCL:** a higher `alpha1` with fewer particles (e.g. 300–800) and `max_beams` 60, measured against the scan match. Or use a scan-to-map correction at rest.
4. **Lidar yaw:** re-check the +1.6° on longer straight runs.
