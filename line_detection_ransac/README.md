# Line Detection RANSAC

## Overview

The `line_detection_ransac` package provides line detection functionality using the a custom variant of ransac for images. It is designed to detect linear features seen from the origin of a sonar image using rays and a custom ransac algorithm

## Architecture

The package follows the standard Vortex separation of concerns:

```
line_detection_ransac/
├── include/
│   └── line_detection_ransac
│       ├── lib/                    # Non-ROS library headers
│       │   └── ransac_line_detection.hpp
│       └── ros/                    # ROS node headers
│           └── line_detection_ros.hpp
├── src/
│   ├── lib/                        # Non-ROS library implementations
│   │   └── ransac_line_detection.cpp
│   └── ros/                        # ROS node implementations
│       └── line_detection_ros.cpp
├── launch/
│   └── line_detection.launch.py
├── config/
│   └── line_detection_params.yaml
├── test/
│   └── test_ransac_line_detection.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

**lib/** contains pure C++/OpenCV logic with no ROS dependencies. This makes it testable and reusable outside of ROS.

**ros/** contains the ROS 2 node that wraps the library, handling subscriptions, publishers, parameters, and message conversions.

## Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| Input image | `sensor_msgs/msg/Image` | Input image for line detection (8-bit, single or multi-channel) |

### Publications

| Topic | Type | Modes | Description |
|-------|------|-------|-------------|
| Detected lines | `vortex_msgs/msg/LineSegment2DArray` | `standard` `visualize` `debug` | Detected line segments (Always published). |
| Color overlay | `sensor_msgs/msg/Image` (BGR, 8UC3) | `visualize` `debug` | Input color image with detected line segments drawn on top |
| boundary point map | `sensor_msgs/msg/Image` (mono, 8UC1) | `debug` | boundary point map |
| boundary overlay | `sensor_msgs/msg/Image` (BGR, 8UC3) | `debug` | boundary map with detected line segments drawn on top |


## Parameters

### boundary Detection

The boundary detector casts rays from the origin of the sonar image and calculates the average intensity of the surrounding pixels along the ray until it hits a threshold where it places a point

| Parameter | Type | Description |
|-----------|------|-------------|
| `threshold` | `int` | Intensity threshold before it places a point |
| `step` | `double` | the step length along a ray in pixel lengths |
| `rays` | `int` | the amount of rays to cast |
| `sample size` | `int` | The side length of the sample square for each point along a ray |
| `angle` | `int` | the FOV of the sonar image |

### Custom RANSAC algorithm

What makes it custom is that instead of random point sampling it iterates through all detected boundary points and check lines from following points because all points on a wall will be next to each other in the container

| Parameter | Type | Description |
|-----------|------|-------------|
| `points_checked` | `int` | number of points along an iteration to check for lines |
| `distance_threshold` | `double` | the distance from a line a point has to be to be considered an inlier |
| `min_remaining_points` | `int` | Minimum number of points remaining to continue RANSAC iterations |
| `min_inliers` | `double` | minimum inliers in the best detected line to continue iterations |



> **Note:** Adjust parameters in the config YAML or via `ros2 param set` at runtime.

## Build

```bash
# Build this package and its dependencies
colcon build --packages-up-to line_detection_ransac --symlink-install
source install/setup.bash
```

## Run

```bash
# Launch the node with default parameters
ros2 launch line_detection_ransac line_detection_ransac.launch.py
```


## Theory

The [Random sample consensus](https://en.wikipedia.org/wiki/Random_sample_consensus)

otherwise vibes
