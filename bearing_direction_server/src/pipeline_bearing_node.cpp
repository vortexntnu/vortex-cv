#include "bearing_direction_server/pipeline_bearing_node.hpp"

#include <spdlog/spdlog.h>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace bearing_direction_server {

PipelineBearingNode::PipelineBearingNode(const rclcpp::NodeOptions& options)
    : BearingDirectionBase("pipeline_bearing_server", options) {
    declare_parameter<std::string>("topics.pixel_detections",
                                   "pipeline/pixel_detections");
    declare_parameter<std::string>("topics.camera_info",
                                   "pipeline/camera_info");

    const auto qos = vortex::utils::qos_profiles::sensor_data_profile(1);

    pixel_sub_ = create_subscription<vortex_msgs::msg::Point2DArray>(
        get_parameter("topics.pixel_detections").as_string(), qos,
        std::bind(&PipelineBearingNode::pixel_callback, this,
                  std::placeholders::_1));

    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("topics.camera_info").as_string(),
        rclcpp::QoS(1).best_effort(),
        std::bind(&PipelineBearingNode::camera_info_callback, this,
                  std::placeholders::_1));

    setup_base();
}

void PipelineBearingNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (intrinsics_) return;  // set once
    intrinsics_ = CameraIntrinsics{
        .fx = msg->k[0], .fy = msg->k[4],
        .cx = msg->k[2], .cy = msg->k[5]};
    spdlog::info("PipelineBearingNode: got camera intrinsics "
                 "fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}",
                 intrinsics_->fx, intrinsics_->fy,
                 intrinsics_->cx, intrinsics_->cy);
}

void PipelineBearingNode::pixel_callback(
    const vortex_msgs::msg::Point2DArray::SharedPtr msg) {
    {
        std::lock_guard lock(mutex_);
        if (!collecting_) return;
    }
    if (!intrinsics_) return;
    if (msg->points.empty()) return;

    const std::string& cam_frame = msg->header.frame_id;
    if (cam_frame.empty()) {
        spdlog::warn("PipelineBearingNode: empty frame_id on pixel message.");
        return;
    }

    for (const auto& pt : msg->points) {
        const Eigen::Vector3d dir_cam(
            (pt.x - intrinsics_->cx) / intrinsics_->fx,
            (pt.y - intrinsics_->cy) / intrinsics_->fy,
            1.0);

        const auto dir_odom = rotate_to_odom(
            dir_cam.normalized(), cam_frame,
            rclcpp::Time(msg->header.stamp));
        if (dir_odom) add_direction(*dir_odom);
    }
}

}  // namespace bearing_direction_server
