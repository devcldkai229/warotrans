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

---

# Part 2 (afternoon): BNO055 reconnected, EKF, rotation centre, endpoint geometry

Logs are on the Pi in `~/wt_logs/calib2/` (`motion.jsonl`, `nav.jsonl`, `nav_path.jsonl`). The ground truth is the scan match, as in Part 1.

## IMU and EKF (adopted)
- **BNO055 standing still.** It answers at 0x29 (IMUPLUS, about 42 Hz). Yaw drift over 300 s was 0.000°. Odometry drift was 0. AMCL was 1.4 mm / 0.09° off the truth.
- **Turns with valid truth.** For the two turns whose scan match was trustworthy, the ratio to the truth was:

  | Turn | IMU | Wheel odometry |
  |---|---|---|
  | −90° | 0.993 | 1.255 |
  | +180° | 0.987 | 0.956 |

  Two later EKF runs gave IMU ratios of 0.988 and 0.991. The IMU heading is within about 1 %; wheel heading is wrong by up to 25 %.
- **EKF.** `ekf.yaml` now fuses yaw and yaw rate only from the IMU; odometry contributes vx and vy ("E2"). The wheel yaw rate was dropped because of the slip.
- **AMCL.** `amcl.yaml` is the EKF variant: 300–800 particles and 60 beams, for the Pi CPU.
- **Result.**
  - After ±90° turns, AMCL was 0.023–0.043 m and ≤ 2.4° off the truth. In Part 1, without the IMU, it was 0.07–0.25 m and 5–18°.
  - One Nav2 leg ended with a heading error of 4.8°; AMCL heading was 0.6° off the truth.
- **Launch script.** `pi_run_semantic.sh` now defaults to `EKF=true`. Its stop step also kills `battery_monitor`, `recovery_guard`, `bno055` and `ekf`. Its pkill patterns are bracketed so an SSH command line that names them does not kill itself.

## Turning centre in the shifted frame (base_footprint x = 0.12, tried on the Pi, not committed)
- **CCW turns.** The base moved 0.032–0.036 m per 90°. The fixed point was (−0.02, −0.01) m in the new frame, so the 0.12 m offset is right.
- **CW turns.** The base still moved 0.157 m, the fixed point was 0.14 m to the left, and the robot turned 69° for 90° of wheel odometry. Traction is uneven between the two sides; this is mechanical (tyres, weight, track). The IMU keeps the heading right anyway.

## Endpoint geometry: the blocking finding
Turning about a point 0.12 m behind the chassis centre, the padded footprint sweeps a radius of 0.335 m. Nav2 with the old frame assumed 0.21 m.

The table checks the padded footprint along the turning arc. Each endpoint is approached at heading 0°, 90°, 180° or −90° and then turns to its own heading. ok means a gap of at least 2 cm.

| Endpoint (yaw) | from 0° | from 90° | from 180° | from −90° |
|---|---|---|---|---|
| INBOUND (180) | HIT | ok | ok | ok |
| QUEUE-POINT (−90) | HIT | HIT | HIT | ok |
| SHELF-B-01 (90) | HIT | ok | ok 0.04 | HIT |
| SHELF-B-02 (0) | ok | HIT | HIT | HIT |
| SHELF-A-01 (90) | HIT | ok | HIT | HIT |
| SHELF-C-01, OUTBOUND, INSPECTION (0) | ok (0.04–0.07) | HIT | HIT | HIT |
| PARKING-A (180) | HIT | ok 0.03 | ok | ok 0.03 |
| PARKING-B (180) | HIT | HIT | ok | HIT |

**What it means.**
- Most endpoints can be reached only by driving straight in, already at the endpoint heading.
- Nav2 plans to the endpoint position and then turns in place.
- With the true geometry, Nav2 refuses those turns: "collision ahead", recoveries, timeout. This happened on INBOUND when the robot arrived heading 0°: 8 recoveries, then a timeout.
- With the old geometry, Nav2 believed the turns fit. The real robot then swung its nose or tail 12 cm wider than planned. This is the "cuts into the neighbouring lane" and "turns forever and does not get in" seen on 2026-11-10.

A pre-pose with a full 360° turn does not fit anywhere either: the lanes are about 0.66 m wide, and the turning circle is 0.67 m across.

## Pi CPU
With the stack idle (Nav2, AMCL, EKF, IMU, no gateway), the Pi uses about 44 % CPU at a load average of about 10. Most nodes take 7–13 % each. Restricting DDS discovery to localhost made no difference, and there is no thermal throttling. Scan-match scripts on top of this caused the Wi-Fi lag.

## State left on the robot
- **Running config:** EKF E2, light AMCL and the wheel PI.
- **Frame:** geometry restored to x = 0, footprint restored.
- **Running now:** the stack, with `MOTORS=false`.
- **Gateway:** has the goal offset (from TF, so 0 today). 371 tests pass on the Pi; the ROS integration test passes on its own domain and fails only while the real stack is running on domain 0.
- **Backup:** `~/warotrans_backups/ros2_ws_src_20261111_140508_precalib2.tgz`.

## Decision needed before the robot can be "100 %"
The robot's real turning geometry does not fit the current endpoint and lane layout. Options:
1. **Mechanical.** Make the robot turn about its chassis centre: balance the weight between the axles, or fix the uneven traction (CW vs CCW). Then the old frame becomes true and every endpoint above fits as Nav2 assumed.
2. **Docking approach in software.**
   - Give each endpoint an approach pose on its lane, reached at the endpoint heading. The robot then drives straight in, and backs out to leave.
   - Adopt x = 0.12 (the gateway offset and the rig support are already in place).
   - This needs approach poses per endpoint (map V007) and NavigateThroughPoses or a short straight-in controller.
3. **Map V007.** Move endpoints away from walls where a single 90° turn has room, for example QUEUE-POINT, SHELF-C-01, OUTBOUND, INSPECTION.

Proposed coordinates come from `calib_epclear.py` / `calib_arc.py` and wait for the operator's approval.

---

# Part 3 (15:00–16:00): operator geometry, ticks, turning centre

## Operator measurements (tape)
- **Drive.** 4 wheels, 4 motors (one per corner). Encoders only on the two front wheels.
- **Track and wheelbase.** Track (centre to centre), front and rear: 0.214 m. Wheelbase (front to rear axle): 0.184 m.
- **Lidar position.** Over the front axle (0 m), 0.18 m ahead of the rear axle, 0.10 m ahead of the chassis centre, no lateral offset.
- **Ticks per wheel turn.** Robot pushed on the floor, 10 turns of the watched front wheel counted:

  | Wheel | Measured | Was |
  |---|---|---|
  | Left | 2481.9 | 2478.1 |
  | Right | 2468.6 | 2478.1 |

## What this explains
- `geometry.yaml` had the lidar 0.01 m ahead of `base_link`, but it is 0.10 m ahead of the chassis centre. Every pose the stack used was the lidar point, about 10 cm ahead of the body.
- That is most of the "turning centre 12 cm behind" from Parts 1 and 2.
- The Part 2 endpoint blocker assumed a 0.335 m swept radius, which came from that error. It no longer holds as stated.

## Adopted (geometry.yaml, nav2_params.yaml, hardware.yaml)
- **Lidar.** Laser x = 0.10 in base_link.
- **Turning centre.** `base_footprint_to_base_link.x = 0.05`: base_footprint is the measured turning centre. CCW turns put it 0.13–0.17 m behind the lidar over three sessions, near the rear axle; weight sits on the rear axle.
- **Footprint.** Centred on the chassis, `[[0.21, 0.13], [0.21, -0.13], [-0.11, -0.13], [-0.11, 0.13]]` around base_footprint. Swept radius with padding is 0.28 m.
- **Ticks per turn.** As measured above. Wheel radii are unchanged; straight-line odometry/true was 0.99–1.03.
- **Gateway goal offset.** Read from TF it is now 0.05 m. Goals are moved back, and proposed plans forward, by that much, so endpoints stay the chassis centre.

## Validation turns, x = 0.02 trial (before the final 0.05)
CCW turns, +90° and +180°:
- Fixed point (−0.057, +0.022) and (−0.034, +0.004) m.
- Base moved 0.067–0.083 m.
- IMU/true ratio 1.02 and 0.994.

CW turns: the fixed point is 0.11–0.13 m to the left and the robot turns 64–147° for 90–180° of wheel odometry. Uneven traction left/right (mechanical); the IMU keeps the heading right.

## Final validation, x = 0.05 (CCW +90° twice)

| | Turn 1 | Turn 2 |
|---|---|---|
| Fixed point | (−0.020, +0.017) m | (−0.015, +0.018) m |
| Base moved | 0.038 m | 0.033 m |
| IMU/true | 0.994 | 0.994 |
| Wheel odometry/true | 0.986 | 0.984 |
| AMCL after nomotion | 0.054 m / 0.1° | 0.072 m / 2.4° |

The turning centre is now within 2 cm of base_footprint. Before the correction the base moved 0.14–0.22 m per 90° turn.

**Still open.**
- **CW traction.** CW turns still pivot 0.11–0.13 m to the left. This is mechanical: check the left-side motors' reverse torque or the tyres.
- **Nav2 and TR.** The full-map Nav2 route and the backend TR acceptance run were not done in this session (time).
- **Snapshot.** The robot was left locked: `MOTORS=false`, EKF on, these files on the Pi.
