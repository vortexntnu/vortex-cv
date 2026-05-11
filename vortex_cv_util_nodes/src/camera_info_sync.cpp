#include "vortex_cv_util_nodes/camera_info_sync.hpp"

#include <yaml-cpp/yaml.h>

#include <rclcpp_components/register_node_macro.hpp>

namespace vortex_cv_util_nodes {

CameraInfoSync::CameraInfoSync(const rclcpp::NodeOptions& options)
    : Node("camera_info_sync", options) {
    const auto file_path = declare_parameter<std::string>("camera_info_file");
    const auto topic_name = declare_parameter<std::string>("camera_info_topic");
    const auto image_topic = declare_parameter<std::string>("image_topic");

    const YAML::Node data = YAML::LoadFile(file_path);

    camera_info_.width = data["image_width"].as<uint32_t>();
    camera_info_.height = data["image_height"].as<uint32_t>();
    camera_info_.distortion_model = data["distortion_model"].as<std::string>();

    const auto k_vec = data["camera_matrix"]["data"].as<std::vector<double>>();
    std::copy(k_vec.begin(), k_vec.end(), camera_info_.k.begin());

    camera_info_.d =
        data["distortion_coefficients"]["data"].as<std::vector<double>>();

    const auto r_vec =
        data["rectification_matrix"]["data"].as<std::vector<double>>();
    std::copy(r_vec.begin(), r_vec.end(), camera_info_.r.begin());

    const auto p_vec =
        data["projection_matrix"]["data"].as<std::vector<double>>();
    std::copy(p_vec.begin(), p_vec.end(), camera_info_.p.begin());

    const auto reliable_qos = rclcpp::QoS(10).reliable();

    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic_name,
                                                               reliable_qos);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, reliable_qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            image_callback(msg);
        });

    RCLCPP_INFO(get_logger(), "Publishing CameraInfo on %s synced to %s",
                topic_name.c_str(), image_topic.c_str());
    RCLCPP_INFO(get_logger(), "Loaded calibration: %s", file_path.c_str());
}

void CameraInfoSync::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    camera_info_.header.stamp = msg->header.stamp;
    camera_info_.header.frame_id = msg->header.frame_id;
    info_pub_->publish(camera_info_);
}

}  // namespace vortex_cv_util_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(vortex_cv_util_nodes::CameraInfoSync)
