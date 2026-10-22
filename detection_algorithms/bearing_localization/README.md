# Bearing Localization

ROS 2 node that estimates the 3D position of a target using bearing-only triangulation. It collects directional measurements from different viewpoints and solves for the point in space that best fits all rays.


## Topics

| Direction | Parameter | Default | Type |
|---|---|---|---|
| Sub | `topics.bearing_measurement` | `bearing_measurement` | `geometry_msgs/Vector3Stamped` |
| Sub | `topics.bearing_array` | `/aruco_detector/marker_directions` | `vortex_msgs/Vector3Array` |
| Pub | `topics.landmarks` | `landmarks` | `vortex_msgs/LandmarkArray` |
| Pub | `topics.bearing_localization_markers` | `bearing_localization/markers` | `visualization_msgs/MarkerArray` |

## Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `profile` | `default` | Parameter profile (`default`, `aruco`) |
| `target_frame` | `orca/odom` | Reference frame for output |
| `window_size` | 30 | Max buffered measurements |
| `max_measurement_age_sec` | 5.0 | Measurement expiry (seconds) |
| `min_measurements` | 5 | Min rays to attempt solve |
| `min_baseline_m` | 0.5 | Min distance between viewpoints |
| `min_ray_angle_deg` | 5.0 | Min angular spread between rays |
| `outlier_residual_threshold_m` | 1.0 | Outlier rejection threshold |
