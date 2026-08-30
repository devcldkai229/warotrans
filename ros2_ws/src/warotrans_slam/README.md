# warotrans_slam

Owns SLAM Toolbox mapping configuration and launch.

Expected inputs:

```text
/scan
/odom
TF: odom -> base_footprint -> base_link -> laser
```

Expected outputs:

```text
/map
TF: map -> odom
```

This package must not parse encoder serial data.
