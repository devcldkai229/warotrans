# warotrans_description

Owns robot geometry and static TF relationships.

## May contain

- `base_footprint`
- `base_link`
- wheel links/joints
- `laser`
- `camera_link`
- meshes and visual/collision geometry

## Must not contain

- serial parsing
- encoder calculation
- odometry integration
- SLAM/Nav2 logic

## Critical rule

Sensor poses come from physical measurement, not estimation.

Expected TF branch after measurements:

```text
base_footprint
  └── base_link
       ├── laser
       └── camera_link
```
