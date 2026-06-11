#include "bearing_direction_server/platform_bearing_node.hpp"

#include <algorithm>
#include <spdlog/spdlog.h>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace bearing_direction_server {

PlatformBearingNode::PlatformBearingNode(const rclcpp::NodeOptions& options)
    : BearingDirectionBase("platform_bearing_server", options) {
    declare_parameter<std::string>("topics.detections",
                                   "platform/detections");
    declare_parameter<std::string>("topics.camera_info",
                                   "platform/camera_info");

    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        get_parameter("topics.detections").as_string(),
        vortex::utils::qos_profiles::sensor_data_profile(1),
        std::bind(&PlatformBearingNode::detection_callback, this,
                  std::placeholders::_1));

    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("topics.camera_info").as_string(),
        rclcpp::QoS(1).best_effort(),
        std::bind(&PlatformBearingNode::camera_info_callback, this,
                  std::placeholders::_1));

    setup_base();
}

void PlatformBearingNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (intrinsics_) return;
    intrinsics_ = CameraIntrinsics{
        .fx = msg->k[0], .fy = msg->k[4],
        .cx = msg->k[2], .cy = msg->k[5]};
    spdlog::info("PlatformBearingNode: got camera intrinsics "
                 "fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}",
                 intrinsics_->fx, intrinsics_->fy,
                 intrinsics_->cx, intrinsics_->cy);
}

void PlatformBearingNode::detection_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    {
        std::lock_guard lock(mutex_);
        if (!collecting_) return;
    }
    if (!intrinsics_) return;
    if (msg->detections.empty()) return;

    const std::string& cam_frame = msg->header.frame_id;
    if (cam_frame.empty()) {
        spdlog::warn("PlatformBearingNode: empty frame_id on detection message.");
        return;
    }

    // Use highest-confidence detection
    const auto& best = *std::max_element(
        msg->detections.begin(), msg->detections.end(),
        [](const auto& a, const auto& b) {
            const double sa = a.results.empty() ? 0.0 : a.results.front().hypothesis.score;
            const double sb = b.results.empty() ? 0.0 : b.results.front().hypothesis.score;
            return sa < sb;
        });

    const double u = best.bbox.center.position.x;
    const double v = best.bbox.center.position.y;

    const Eigen::Vector3d dir_cam(
        (u - intrinsics_->cx) / intrinsics_->fx,
        (v - intrinsics_->cy) / intrinsics_->fy,
        1.0);

    const auto dir_odom = rotate_to_odom(
        dir_cam.normalized(), cam_frame,
        rclcpp::Time(msg->header.stamp));
    if (dir_odom) {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        {
            std::lock_guard lock(mutex_);
            if (latest_drone_pos_)
                pos = Eigen::Vector3d(latest_drone_pos_->x, latest_drone_pos_->y, latest_drone_pos_->z);
        }
        add_direction(*dir_odom, pos);
    }
}

}  // namespace bearing_direction_server
