#ifndef BEARING_DIRECTION_SERVER__PLATFORM_BEARING_NODE_HPP_
#define BEARING_DIRECTION_SERVER__PLATFORM_BEARING_NODE_HPP_

#include "bearing_direction_server/bearing_direction_base.hpp"
#include "bearing_direction_server/pipeline_bearing_node.hpp"  // CameraIntrinsics

#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace bearing_direction_server {

/**
 * Subscribes to Detection2DArray (YOLO OBB output) and CameraInfo.
 * Picks the highest-confidence detection, back-projects its bounding-box
 * centre to a ray in the camera frame, then rotates it into odom via TF.
 */
class PlatformBearingNode : public BearingDirectionBase {
   public:
    explicit PlatformBearingNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void detection_callback(
        const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    std::optional<CameraIntrinsics> intrinsics_;
};

}  // namespace bearing_direction_server

#endif  // BEARING_DIRECTION_SERVER__PLATFORM_BEARING_NODE_HPP_
