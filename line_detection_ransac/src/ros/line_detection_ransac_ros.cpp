#include "line_detection_ransac/ros/line_detection_ransac_ros.hpp"
#include <cv_bridge/cv_bridge.h>
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "line_detection_ransac/lib/typedefs.hpp"

namespace vortex::line_detection {

LineDetectionRansacNode::LineDetectionRansacNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("line_detection_ransac_node", options) {
    declare_parameters();
    setup_publishers_and_subscribers();
    set_detector();
    set_mode();
    initialize_parameter_handler();
}

void LineDetectionRansacNode::declare_parameters() {
    this->declare_parameter<std::string>("topic.image_sub_topic");
    this->declare_parameter<std::string>("topic.line_segments_pub_topic");
    this->declare_parameter<std::string>("topic.color_overlay_pub_topic");
    this->declare_parameter<std::string>("topic.boundary_debug_pub_topic");
    this->declare_parameter<std::string>("topic.boundary_overlay_pub_topic");

    this->declare_parameter<int>("boundary_detection.threshold");
    this->declare_parameter<float>("boundary_detection.step");
    this->declare_parameter<int>("boundary_detection.sample_side_length");
    this->declare_parameter<int>("boundary_detection.angle");
    this->declare_parameter<int>("boundary_detection.num_rays");
    this->declare_parameter<bool>("boundary_detection.edge_detection");

    this->declare_parameter<int>("ransac.points_checked");
    this->declare_parameter<float>("ransac.inlier_threshold");
    this->declare_parameter<int>("ransac.min_remaining_points");
    this->declare_parameter<int>("ransac.min_inliers");
    this->declare_parameter<float>("ransac.max_distance");

    this->declare_parameter<std::string>("mode");
}

void LineDetectionRansacNode::setup_publishers_and_subscribers() {
    const std::string image_sub_topic =
        this->get_parameter("topic.image_sub_topic").as_string();
    const std::string line_segments_pub_topic =
        this->get_parameter("topic.line_segments_pub_topic").as_string();
    const std::string color_overlay_pub_topic =
        this->get_parameter("topic.color_overlay_pub_topic").as_string();
    const std::string boundary_debug_pub_topic =
        this->get_parameter("topic.boundary_debug_pub_topic").as_string();
    const std::string boundary_overlay_pub_topic =
        this->get_parameter("topic.boundary_overlay_pub_topic").as_string();

    const auto qos_profile =
        vortex::utils::qos_profiles::sensor_data_profile(1);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_sub_topic, qos_profile,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            this->image_callback(msg);
        });

    line_segments_pub_ =
        this->create_publisher<vortex_msgs::msg::LineSegment2DArray>(
            line_segments_pub_topic, qos_profile);

    color_overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        color_overlay_pub_topic, qos_profile);

    boundary_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        boundary_debug_pub_topic, qos_profile);

    boundary_overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        boundary_overlay_pub_topic, qos_profile);
}

void LineDetectionRansacNode::set_detector() {
    BoundaryConfig boundary_config;
    boundary_config.threshold =
        this->get_parameter("boundary_detection.threshold").as_int();
    boundary_config.step =
        this->get_parameter("boundary_detection.step").as_double();
    boundary_config.sample_side_length =
        this->get_parameter("boundary_detection.sample_side_length").as_int();
    boundary_config.angle =
        this->get_parameter("boundary_detection.angle").as_int();
    boundary_config.num_rays =
        this->get_parameter("boundary_detection.num_rays").as_int();
    boundary_config.edge_detection =
        this->get_parameter("boundary_detection.edge_detection").as_bool();

    RansacConfig ransac_config;
    ransac_config.points_checked =
        this->get_parameter("ransac.points_checked").as_int();
    ransac_config.inlier_threshold =
        this->get_parameter("ransac.inlier_threshold").as_double();
    ransac_config.min_remaining_points =
        this->get_parameter("ransac.min_remaining_points").as_int();
    ransac_config.min_inliers =
        this->get_parameter("ransac.min_inliers").as_int();
    ransac_config.max_distance =
        this->get_parameter("ransac.max_distance").as_double();

    detector_ =
        std::make_unique<LineDetectorRansac>(boundary_config, ransac_config);
}

void LineDetectionRansacNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception& e) {
        spdlog::error("cv_bridge exception: {}", e.what());
        return;
    }

    const cv::Mat& input_image = cv_ptr->image;

    Result result;
    try {
        result = detector_->detect(input_image, mode_);
    } catch (const std::exception& e) {
        spdlog::error("Line detection failed: {}", e.what());
        return;
    }

    vortex_msgs::msg::LineSegment2DArray line_segments_msg;
    line_segments_msg.header = msg->header;
    line_segments_msg.lines.reserve(result.line_segments.size());

    for (const auto& segment : result.line_segments) {
        vortex_msgs::msg::LineSegment2D seg_msg;
        seg_msg.p0.x = segment.p0.x;
        seg_msg.p0.y = segment.p0.y;
        seg_msg.p1.x = segment.p1.x;
        seg_msg.p1.y = segment.p1.y;
        line_segments_msg.lines.push_back(seg_msg);
    }
    line_segments_pub_->publish(line_segments_msg);

    mode_conditional_publishing(result, msg->header);
}

RCLCPP_COMPONENTS_REGISTER_NODE(LineDetectionRansacNode)

}  // namespace vortex::line_detection
