#ifndef DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_HPP_
#define DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_HPP_

#include <cmath>

namespace vortex::docking_camera_yolo_direction_waypoint {

/**
 * @brief Camera intrinsic parameters needed for yaw extraction.
 *
 * Sourced from the 3x3 camera matrix K in sensor_msgs/CameraInfo:
 *   K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
 */
struct CameraIntrinsics {
    double fx;  //< Focal length in x [pixels]
    double cx;  //< Principal point x [pixels]
    double cy;  //< Principal point y [pixels]
};

/**
 * @brief Computes the yaw angle of a detected object relative to the camera's
 * optical axis.
 *
 * Uses the image coordinate convention: +X right, +Y down (sensor_msgs/Image).
 * The direction vector from the image centre to the bounding-box centre is
 * projected onto the horizontal axis, then atan2 is applied with the focal
 * length to obtain the true angular deviation in radians.
 *
 * Positive yaw means the object is to the right of the optical axis.
 */
class YawExtractor {
   public:
    /**
     * @brief Compute the horizontal yaw from a bounding-box centre pixel.
     *
     * @param bbox_center_x  Bounding-box centre x coordinate [pixels].
     * @param intrinsics     Camera intrinsic parameters.
     * @return Yaw angle [rad] in (-π/2, π/2); positive = object is right of
     *         centre.
     */
    double compute_yaw(double bbox_center_x,
                       const CameraIntrinsics& intrinsics) const;
};

}  // namespace vortex::docking_camera_yolo_direction_waypoint

#endif  // DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_HPP_
