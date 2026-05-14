#include <docking_camera_yolo_direction_waypoint/docking_camera_yolo_direction_waypoint_ros.hpp>

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace vortex::docking_camera_yolo_direction_waypoint {

DockingCameraYoloDirectionWaypointNode::DockingCameraYoloDirectionWaypointNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("docking_camera_yolo_direction_waypoint_node", options) {
    setup_parameters();
    setup_publishers_and_subscribers();
}

void DockingCameraYoloDirectionWaypointNode::setup_parameters() {
    detection_sub_topic_  = this->declare_parameter<std::string>("detection_sub_topic");
    camera_info_sub_topic_ = this->declare_parameter<std::string>("camera_info_sub_topic");
    landmarks_pub_topic_  = this->declare_parameter<std::string>("landmarks_pub_topic");
    odom_frame_           = this->declare_parameter<std::string>("odom_frame");
    waypoint_distance_    = this->declare_parameter<double>("waypoint_distance");
}

void DockingCameraYoloDirectionWaypointNode::setup_publishers_and_subscribers() {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::SensorDataQoS();

    detection_sub_ =
        this->create_subscription<vision_msgs::msg::Detection2DArray>(
            detection_sub_topic_, qos,
            [this](const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg) {
                detection_callback(msg);
            });

    camera_info_sub_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_sub_topic_, rclcpp::QoS(1).best_effort(),
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
                camera_info_callback(msg);
            });

    landmarks_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkArray>(
            landmarks_pub_topic_, rclcpp::QoS(10).best_effort());
}

void DockingCameraYoloDirectionWaypointNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (intrinsics_) {
        return;
    }

    // K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    intrinsics_ = CameraIntrinsics{
        .fx = msg->k[0],
        .cx = msg->k[2],
        .cy = msg->k[5],
    };
    camera_frame_ = msg->header.frame_id;

    RCLCPP_INFO(this->get_logger(),
                "Camera intrinsics received: fx=%.2f cx=%.2f cy=%.2f frame='%s'",
                intrinsics_->fx, intrinsics_->cx, intrinsics_->cy,
                camera_frame_.c_str());
}

std::optional<geometry_msgs::msg::TransformStamped>
DockingCameraYoloDirectionWaypointNode::lookup_camera_transform_in_odom(
    const rclcpp::Time& stamp) {
    try {
        return tf_buffer_->lookupTransform(odom_frame_, camera_frame_,
                                           stamp, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "TF %s -> %s failed: %s", odom_frame_.c_str(),
                             camera_frame_.c_str(), ex.what());
        return std::nullopt;
    }
}

void DockingCameraYoloDirectionWaypointNode::detection_callback(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg) {
    if (!intrinsics_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No camera intrinsics yet — skipping detection.");
        return;
    }

    if (msg->detections.empty()) {
        return;
    }

    // Pick the detection with the highest score.
    const auto& best = *std::max_element(
        msg->detections.begin(), msg->detections.end(),
        [](const vision_msgs::msg::Detection2D& a,
           const vision_msgs::msg::Detection2D& b) {
            const double score_a =
                a.results.empty() ? 0.0 : a.results.front().hypothesis.score;
            const double score_b =
                b.results.empty() ? 0.0 : b.results.front().hypothesis.score;
            return score_a < score_b;
        });

    const auto tf_opt =
        lookup_camera_transform_in_odom(rclcpp::Time(msg->header.stamp));
    if (!tf_opt) {
        return;
    }

    // Extract camera heading in odom from the TF rotation.
    tf2::Quaternion q;
    tf2::fromMsg(tf_opt->transform.rotation, q);
    double roll, pitch, yaw_camera_in_odom;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_camera_in_odom);

    // Camera-relative yaw from bbox pixel offset.
    const double bbox_center_x      = best.bbox.center.position.x;
    const double yaw_camera_relative = extractor_.compute_yaw(bbox_center_x, *intrinsics_);

    // Compose and wrap to (-π, π].
    double yaw_target_odom = yaw_camera_in_odom + yaw_camera_relative;
    yaw_target_odom =
        std::atan2(std::sin(yaw_target_odom), std::cos(yaw_target_odom));

    // Camera position in odom — project the waypoint from here.
    const double cam_x = tf_opt->transform.translation.x;
    const double cam_y = tf_opt->transform.translation.y;

    const double wx = cam_x + waypoint_distance_ * std::cos(yaw_target_odom);
    const double wy = cam_y + waypoint_distance_ * std::sin(yaw_target_odom);

    RCLCPP_DEBUG(this->get_logger(),
                 "cam_yaw_odom=%.3f  yaw_cam_rel=%.3f  yaw_target=%.3f  "
                 "waypoint=(%.2f, %.2f)",
                 yaw_camera_in_odom, yaw_camera_relative, yaw_target_odom, wx, wy);

    publish_landmark(wx, wy, yaw_target_odom);
}

void DockingCameraYoloDirectionWaypointNode::publish_landmark(
    double x, double y, double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    const auto stamp = this->now();

    vortex_msgs::msg::Landmark landmark;
    landmark.header.stamp    = stamp;
    landmark.header.frame_id = odom_frame_;
    landmark.id              = 0;
    landmark.type.value      = vortex_msgs::msg::LandmarkType::ARUCO_BOARD;
    landmark.subtype.value   = 0;
    landmark.pose.pose.position.x  = x;
    landmark.pose.pose.position.y  = y;
    landmark.pose.pose.position.z  = 0.0;
    landmark.pose.pose.orientation = tf2::toMsg(q);

    vortex_msgs::msg::LandmarkArray landmark_array;
    landmark_array.header.stamp    = stamp;
    landmark_array.header.frame_id = odom_frame_;
    landmark_array.landmarks.push_back(landmark);

    landmarks_pub_->publish(landmark_array);

    RCLCPP_INFO(this->get_logger(),
                "Landmark published: x=%.2f y=%.2f yaw=%.3f rad", x, y, yaw);
}

RCLCPP_COMPONENTS_REGISTER_NODE(
    vortex::docking_camera_yolo_direction_waypoint::
        DockingCameraYoloDirectionWaypointNode)

}  // namespace vortex::docking_camera_yolo_direction_waypoint
