#pragma once

#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace vortex_cv_util_nodes {

class ImageUndistort : public rclcpp::Node {
   public:
    explicit ImageUndistort(const rclcpp::NodeOptions& options);

   private:
    void init_maps_from_file(const std::string& path);
    void build_maps(const cv::Mat& k, const cv::Mat& d, int w, int h);
    // Loads k, d, w, h from `path` into `raw_info_from_file_` so passthrough
    // mode publishes calibration-derived CameraInfo instead of relaying
    // upstream values (useful when the bag/camera reports wrong intrinsics).
    void load_raw_info_from_file(const std::string& path);

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void relay_image(const sensor_msgs::msg::Image::SharedPtr msg);
    void relay_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    cv::Mat map1_, map2_;
    sensor_msgs::msg::CameraInfo rectified_info_;
    sensor_msgs::msg::CameraInfo raw_info_from_file_;
    bool maps_ready_{false};
    bool raw_info_from_file_ready_{false};
    std::string output_frame_;
};

}  // namespace vortex_cv_util_nodes
