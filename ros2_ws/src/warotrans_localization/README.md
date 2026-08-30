# warotrans_localization

Owns localization/state-estimation configuration.

Planned stages:

1. wheel odometry baseline
2. BNO055 verification
3. optional `robot_localization` EKF
4. static-map localization (for example AMCL)

Do not enable multiple publishers for:

```text
odom -> base_footprint
```
