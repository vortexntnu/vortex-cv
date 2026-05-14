#include <docking_camera_yolo_direction_waypoint/docking_camera_yolo_direction_waypoint.hpp>

namespace vortex::docking_camera_yolo_direction_waypoint {

double YawExtractor::compute_yaw(double bbox_center_x,
                                 const CameraIntrinsics& intrinsics) const {
    const double dx = bbox_center_x - intrinsics.cx;
    return std::atan2(dx, intrinsics.fx);
}

}  // namespace vortex::docking_camera_yolo_direction_waypoint
