#include <memory>

#include <docking_position_estimator/docking_position_estimator_ros.hpp>

#include <spdlog/spdlog.h>
#include <tf2/time.h>
#include <tf2_ros/create_timer_ros.h>
#include <rclcpp/node_options.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <rclcpp_components/register_node_macro.hpp>
// #include <vortex/utils/ros/qos_profiles.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>  // ERSTATTE MED VORTEX? ^^^
#include <vortex/utils/types.hpp>
#include "docking_position_estimator/docking_position_estimator.hpp"

namespace vortex::docking_position_estimator {

DockingPositionEstimatorNode::DockingPositionEstimatorNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("docking_position_estimator_node", options) {
    setup_parameters();
    setup_publishers_and_subscribers();
    setup_estimator();
}

void DockingPositionEstimatorNode::setup_parameters() {
    // Frames
    odom_frame_ = this->declare_parameter<std::string>("odom_frame");

    // Topics
    line_sub_topic_ = this->declare_parameter<std::string>("line_sub_topic");
    pose_sub_topic_ = this->declare_parameter<std::string>("pose_sub_topic");
    sonar_info_sub_topic_ =
        this->declare_parameter<std::string>("sonar_info_sub_topic");
    debug_topic_ =
        this->declare_parameter<std::string>("debug_topic");  // For testing
}

void DockingPositionEstimatorNode::setup_publishers_and_subscribers() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos_sub =
        rclcpp::SensorDataQoS();  // standard for sensordata, ta inn vortex sin
                                  // i stedet??
    auto sub_options = rclcpp::SubscriptionOptions();

    line_sub_.subscribe(this, line_sub_topic_, qos_sub.get_rmw_qos_profile(),
                        sub_options);

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_sub_topic_, qos_sub,
        std::bind(&DockingPositionEstimatorNode::pose_callback, this,
                  std::placeholders::_1));

    sonar_info_sub_ = this->create_subscription<vortex_msgs::msg::SonarInfo>(
        sonar_info_sub_topic_, qos_sub,
        [this](const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg) {
            this->sonar_info_callback(msg);
        });

    line_filter_ = std::make_shared<
        tf2_ros::MessageFilter<vortex_msgs::msg::LineSegment2DArray>>(
        line_sub_, *tf_buffer_, odom_frame_, 10,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    line_filter_->registerCallback(
        std::bind(&DockingPositionEstimatorNode::line_callback, this,
                  std::placeholders::_1));

    send_pose_client_ = this->create_client<vortex_msgs::srv::SendPose>(
        "/docking_position_estimator/docking_pose");

    docking_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            debug_topic_, 10);  // til testing
}

void DockingPositionEstimatorNode::setup_estimator() {
    DockingPositionEstimatorConfig config{};

    config.min_wall_distance_m =
        this->declare_parameter<double>("min_wall_distance_m");
    config.max_wall_distance_m =
        this->declare_parameter<double>("max_wall_distance_m");
    config.parallel_heading_angle_threshold_rad =
        this->declare_parameter<double>("parallel_heading_angle_threshold_rad");
    config.perpendicular_heading_angle_threshold_rad =
        this->declare_parameter<double>(
            "perpendicular_heading_angle_threshold_rad");

    config.min_corner_angle_rad =
        this->declare_parameter<double>("min_corner_angle_rad");
    config.max_corner_angle_rad =
        this->declare_parameter<double>("max_corner_angle_rad");
    config.side_wall_offset_m =
        this->declare_parameter<double>("side_wall_offset_m");
    config.far_wall_offset_m =
        this->declare_parameter<double>("far_wall_offset_m");
    config.use_left_wall = this->declare_parameter<bool>("use_left_wall");

    estimator_ = std::make_unique<DockingPositionEstimator>(config);

    // FJERNE? TO DO
    config.far_wall_min_x_m =
        this->declare_parameter<double>("far_wall_min_x_m");
    config.right_wall_max_y_m =
        this->declare_parameter<double>("right_wall_max_y_m");
}

void DockingPositionEstimatorNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
    drone_state_.x = msg->pose.pose.position.x;
    drone_state_.y = msg->pose.pose.position.y;
    drone_state_.z = msg->pose.pose.position.z;

    Eigen::Quaterniond q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    Eigen::Vector3d rpy = vortex::utils::math::quat_to_euler(q);
    drone_state_.yaw = rpy.z();
}

void DockingPositionEstimatorNode::sonar_info_callback(
    const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg) {
    latest_sonar_info_ = msg;
}

void DockingPositionEstimatorNode::line_callback(
    const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg) {
    estimate_and_send_docking_waypoint(*msg);
}

void DockingPositionEstimatorNode::estimate_and_send_docking_waypoint(
    const vortex_msgs::msg::LineSegment2DArray& msg) {
    geometry_msgs::msg::TransformStamped tf_stamped;

    try {
        tf_stamped = tf_buffer_->lookupTransform(
            odom_frame_, msg.header.frame_id, msg.header.stamp);
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("[PoolExploration] TF failed {} -> {}: {}",
                     msg.header.frame_id, odom_frame_, ex.what());
        return;
    }

    const Eigen::Affine3d T = tf2::transformToEigen(tf_stamped.transform);
    const Eigen::Matrix4f T_odom_src = T.matrix().cast<float>();

    auto segs = transform_segments_2d(msg, T_odom_src);  // NB ENDRE NAVN <3

    Eigen::Vector2f drone_pos = {drone_state_.x, drone_state_.y};
    float drone_heading = drone_state_.yaw;

    auto corners =
        estimator_->find_corner_estimates(segs, drone_pos, drone_heading);

    if (corners.empty()) {
        spdlog::info(
            "[PoolExploration] No valid corners -> no docking estimate");
        return;
    }

    CornerEstimate best_corner =
        estimator_->select_best_corner(corners, drone_pos);

    Eigen::Vector2f docking =
        estimator_->estimate_docking_position(best_corner, drone_pos);

    publish_docking_marker(docking);  // til testing
    send_docking_waypoint(docking);

    spdlog::info("[PoolExploration] Docking estimate (odom): x={} y={}",
                 docking.x(), docking.y());
}

std::vector<vortex::utils::types::LineSegment2D>
DockingPositionEstimatorNode::transform_segments_2d(
    const vortex_msgs::msg::LineSegment2DArray& msg,
    const Eigen::Matrix4f& T_target_src) {
    std::vector<vortex::utils::types::LineSegment2D> segments;
    segments.reserve(msg.lines.size());

    if (!latest_sonar_info_) {
        spdlog::warn("[PoolExploration] No sonar image info available yet");
        return segments;
    }

    vortex::utils::types::SonarInfo sonar_info;
    sonar_info.meters_per_pixel_x = latest_sonar_info_->meters_per_pixel_x;
    sonar_info.meters_per_pixel_y = latest_sonar_info_->meters_per_pixel_y;
    sonar_info.image_width = latest_sonar_info_->width;
    sonar_info.image_height = latest_sonar_info_->height;

    for (const auto& line : msg.lines) {
        const auto p0 =
            sonar_info.pixel_index_to_sonar_metric(line.p0.x, line.p0.y);
        const auto p1 =
            sonar_info.pixel_index_to_sonar_metric(line.p1.x, line.p1.y);

        const Eigen::Vector4f p0_sonar(static_cast<float>(p0.y),
                                       static_cast<float>(p0.x), 0.0f, 1.0f);
        const Eigen::Vector4f p1_sonar(static_cast<float>(p1.y),
                                       static_cast<float>(p1.x), 0.0f, 1.0f);

        const Eigen::Vector4f p0_target = T_target_src * p0_sonar;
        const Eigen::Vector4f p1_target = T_target_src * p1_sonar;

        vortex::utils::types::LineSegment2D seg;
        seg.p0 = {p0_target.x(), p0_target.y()};
        seg.p1 = {p1_target.x(), p1_target.y()};

        spdlog::info(
            "[PoolExploration] Line received: ({:.2f}, {:.2f}) -> ({:.2f}, "
            "{:.2f})",  // til debugging
            seg.p0.x, seg.p0.y, seg.p1.x, seg.p1.y);

        segments.push_back(seg);
    }
    return segments;
}

void DockingPositionEstimatorNode::send_docking_waypoint(
    const Eigen::Vector2f& docking_estimate) {
    if (waypoint_sent_) {
        return;
    }

    if (!send_pose_client_->service_is_ready()) {
        spdlog::warn("[PoolExploration] SendPose service not available");
        return;
    }

    auto request = std::make_shared<vortex_msgs::srv::SendPose::Request>();

    request->pose.header.stamp = this->now();
    request->pose.header.frame_id = odom_frame_;
    request->pose.pose.position.x = docking_estimate.x();
    request->pose.pose.position.y = docking_estimate.y();
    request->pose.pose.position.z = 0.0;
    request->pose.pose.orientation.w = 1.0;

    send_pose_client_->async_send_request(request);
    waypoint_sent_ = true;
    spdlog::info("[PoolExploration] Docking pose sent: x={} y={}",
                 docking_estimate.x(), docking_estimate.y());
}

void DockingPositionEstimatorNode::publish_docking_marker(
    const Eigen::Vector2f& docking)  // til testing
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = odom_frame_;
    marker.header.stamp = this->now();

    marker.ns = "docking_debug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = docking.x();
    marker.pose.position.y = docking.y();
    marker.pose.position.z = 0.0;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    docking_marker_pub_->publish(marker);
}

RCLCPP_COMPONENTS_REGISTER_NODE(
    vortex::docking_position_estimator::DockingPositionEstimatorNode)

}  // namespace vortex::docking_position_estimator
