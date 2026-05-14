#ifndef DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_HPP_
#define DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_HPP_

namespace vortex::docking_camera_yolo_direction_waypoint {

// Sourced from the 3x3 camera matrix K in sensor_msgs/CameraInfo:
//   K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
struct CameraIntrinsics {
    double fx;  //< Focal length in x [pixels]
    double fy;  //< Focal length in y [pixels]
    double cx;  //< Principal point x [pixels]
    double cy;  //< Principal point y [pixels]
};

}  // namespace vortex::docking_camera_yolo_direction_waypoint

#endif  // DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_HPP_
