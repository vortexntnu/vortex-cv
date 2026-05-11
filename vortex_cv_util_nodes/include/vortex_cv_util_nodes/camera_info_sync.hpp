#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace vortex_cv_util_nodes {

class CameraInfoSync : public rclcpp::Node {
   public:
    explicit CameraInfoSync(const rclcpp::NodeOptions& options);

   private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace vortex_cv_util_nodes
