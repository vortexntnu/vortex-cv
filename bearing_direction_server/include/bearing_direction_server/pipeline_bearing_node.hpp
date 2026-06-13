#ifndef BEARING_DIRECTION_SERVER__PIPELINE_BEARING_NODE_HPP_
#define BEARING_DIRECTION_SERVER__PIPELINE_BEARING_NODE_HPP_

#include "bearing_direction_server/bearing_direction_base.hpp"

#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vortex_msgs/msg/point2_d_array.hpp>

namespace bearing_direction_server {

struct CameraIntrinsics {
    double fx, fy, cx, cy;
};

/**
 * Subscribes to Point2DArray (pixel coordinates of the pipeline endpoint)
 * and CameraInfo. Converts each pixel to a bearing ray in the camera frame,
 * then rotates it into odom via TF.
 */
class PipelineBearingNode : public BearingDirectionBase {
   public:
    explicit PipelineBearingNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void pixel_callback(const vortex_msgs::msg::Point2DArray::SharedPtr msg);
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Subscription<vortex_msgs::msg::Point2DArray>::SharedPtr pixel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    std::optional<CameraIntrinsics> intrinsics_;
};

}  // namespace bearing_direction_server

#endif  // BEARING_DIRECTION_SERVER__PIPELINE_BEARING_NODE_HPP_
