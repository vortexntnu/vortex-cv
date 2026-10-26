# RoboSub sim helpers

The dummy landmark publisher that used to live here is gone: Stonefish now
publishes the course landmarks itself, as `vortex_msgs/LandmarkArray` on
`/<drone>/landmarks` in `<drone>/odom`, from `<landmark>` tags in the scene
(`vortex-stonefish-sim/stonefish_sim/objects/robosub_landmarks.scn`). The
positions follow the objects, so moving a course element in the scene moves
its landmarks, and the role subtypes follow `robosub_icon_seed` like the
role images do.

What is left in this package:

| Node | What it does |
|---|---|
| `sim_odom_relay_node` | Stands in for the state estimator in the simulator: republishes the sim's odometry as `odom`, `pose`, `twist` and `odom -> base_link` in TF. Run the sim with `mock_odom:=false`. |
| `detections_markers_node` | Landmarks as Foxglove markers (`launch/foxglove_helpers.launch.py`). |

The detection noise profiles (realistic, unstable, erratic) were part of the
dummy publisher and are not in the sim yet.
