# Line Detection HoughP

## Overview

The `line_detection_houghp` package provides line detection functionality using the Probabilistic Hough Transform (HoughP) for images. It is designed to detect linear features such as pipelines, walls, and other structural elements in underwater environments.

## Architecture

The package follows the standard Vortex separation of concerns:

```
line_detection_houghp/
├── include/
│   └── line_detection_houghp/
│       ├── lib/                    # Non-ROS library headers
│       │   └── houghp_line_detection.hpp
│       └── ros/                    # ROS node headers
│           └── line_detection_ros.hpp
├── src/
│   ├── lib/                        # Non-ROS library implementations
│   │   └── houghp_line_detection.cpp
│   └── ros/                        # ROS node implementations
│       └── line_detection_ros.cpp
├── launch/
│   └── line_detection.launch.py
├── config/
│   └── line_detection_params.yaml
├── test/
│   └── test_houghp_line_detection.cpp
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
| Canny edge map | `sensor_msgs/msg/Image` (mono, 8UC1) | `debug` | Raw Canny edge map |
| Canny overlay | `sensor_msgs/msg/Image` (BGR, 8UC3) | `debug` | Edge map converted to BGR with detected line segments drawn on top |


## Parameters

### Canny Edge Detection

The Canny edge detector is used as a pre-processing step to extract edges before the Hough transform. It uses hysteresis thresholding with two thresholds to identify strong and weak edges.

| Parameter | Type | Description |
|-----------|------|-------------|
| `canny_low_threshold` | `int` | Low threshold for the hysteresis procedure. Edges with gradient magnitude below this are discarded. |
| `canny_high_threshold` | `int` | High threshold for the hysteresis procedure. Edges with gradient magnitude above this are always accepted. Edges between low and high are accepted only if connected to a strong edge. |
| `canny_aperture_size` | `int` | Aperture size for the Sobel operator used internally by Canny. Must be odd (3, 5, or 7). Larger values detect smoother edges. |
| `canny_L2_gradient` | `bool` | If `true`, uses the more accurate L2 norm (√(Gx² + Gy²)) for gradient magnitude instead of the default L1 norm (\|Gx\| + \|Gy\|). |

### Probabilistic Hough Transform

| Parameter | Type | Description |
|-----------|------|-------------|
| `hough_rho` | `double` | Distance resolution of the accumulator in pixels |
| `hough_theta` | `double` | Angle resolution of the accumulator in radians |
| `hough_threshold` | `int` | Accumulator threshold — minimum number of votes to accept a line |
| `min_line_length` | `double` | Minimum line length to accept (pixels). Lines shorter than this are rejected. |
| `max_line_gap` | `double` | Maximum allowed gap between points on the same line to link them into a single segment (pixels) |



> **Note:** Adjust parameters in the config YAML or via `ros2 param set` at runtime.

## Build

```bash
# Build this package and its dependencies
colcon build --packages-up-to line_detection_houghp --symlink-install
source install/setup.bash
```

## Run

```bash
# Launch the node with default parameters
ros2 launch line_detection_houghp line_detection.launch.py
```


## Theory

The [Probabilistic Hough Transform](https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga8618180a5948286384e3b7ca02f6feeb) (`cv::HoughLinesP`) is a variant of the Standard Hough Transform that returns line segments rather than infinite lines. It works by:

1. Randomly sampling edge points from a binary image.
2. Accumulating votes in (ρ, θ) parameter space.
3. Extracting line segments that exceed the vote threshold and meet minimum length / maximum gap constraints.

Detected line segments are converted to polar (Hesse normal) form using `vortex::utils::types::LineSegment2D::polar_parametrization()` for downstream consumers.