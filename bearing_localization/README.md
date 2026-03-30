# Bearing Localization

ROS 2 node that estimates the 3D position of targets using bearing-only triangulation. It collects directional measurements from different viewpoints, transforms them into a common frame via TF, and solves for the point in space that best fits all rays. Supports multiple simultaneous targets via per-target localizers.

## Topics

| Direction | Parameter | Default | Type |
|---|---|---|---|
| Sub | `topics.bearing_measurements` | `bearing_measurements` | `vortex_msgs/BearingMeasurementArray` |
| Pub | `topics.landmarks` | `landmarks` | `vortex_msgs/LandmarkArray` |
| Pub | `topics.bearing_localization_markers` | `bearing_localization/markers` | `visualization_msgs/MarkerArray` |

## Algorithm Parameters

Loaded from a profile file (`default.yaml`, `aruco.yaml`, or `pinger.yaml`). Selected via the `profile` launch argument.

| Parameter | Default | Description |
|---|---|---|
| `window_size` | 30 | Max measurements kept in buffer |
| `max_measurement_age_sec` | 5.0 | Discard measurements older than this (s) |
| `min_measurements` | 5 | Min measurements needed to triangulate |
| `min_baseline_m` | 0.5 | Min distance between first and last origin (m) |
| `min_ray_angle_deg` | 5.0 | Min angle between rays to accept solution (deg) |
| `outlier_residual_threshold_m` | 1.0 | Residual above this marks a measurement as outlier (m) |
| `max_outlier_iterations` | 2 | Max rounds of iterative outlier rejection |

## Launch

```bash
# Default profile
ros2 launch bearing_localization bearing_localization.launch.py

# With a specific profile
ros2 launch bearing_localization bearing_localization.launch.py profile:=aruco
```
