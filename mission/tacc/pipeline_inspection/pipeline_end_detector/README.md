# Pipeline End Detector

A ROS2 node that subscribes to the end-of-pipeline classification output and, after a configurable number of consecutive detections, sends a trigger service call to the pipeline inspection FSM to signal that the pipeline has ended.

## Overview

The node listens to a `std_msgs/UInt8` topic published by the end-of-pipeline classifier:

- `data == 1` — Class 1: end of pipeline detected
- `data == 0` — Class 0: continue following

Once `detection_threshold` consecutive `1`s are received, the node calls the `pipeline_inspection_fsm/pipeline_finished` service. If the call fails, the counter resets and the node retries on the next streak.

## ROS2 Interface

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `classification_output` | `std_msgs/UInt8` | End-of-pipeline classification output |

### Service Clients

| Service | Type | Description |
|---|---|---|
| `pipeline_inspection_fsm/pipeline_finished` | `std_srvs/Trigger` | Signals the FSM that the pipeline has ended |

## Configuration

### `pipeline_end_detector_config.yaml`

```yaml
detection_threshold: 10   # consecutive detections required before triggering
topics:
  detection: "classification_output"
  end_of_pipeline_service: "pipeline_inspection_fsm/pipeline_finished"
```

## Running

```bash
ros2 run pipeline_end_detector pipeline_end_detector_node \
  --ros-args --params-file path/to/pipeline_end_detector_config.yaml
```

Or via the launch file:

```bash
ros2 launch pipeline_end_detector pipeline_end_detector.launch.py
```
