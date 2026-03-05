#include "line_detection_houghp/ros/line_detection_houghp_ros.hpp"
#include <cv_bridge/cv_bridge.h>
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "line_detection_houghp/lib/line_detection_houghp.hpp"

namespace vortex::line_detection {

LineDetectionHoughPNode::LineDetectionHoughPNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("line_detection_houghp_node", options) {
  declare_parameters();
  set_detector();
  set_mode();
  setup_publishers_and_subscribers();
  initialize_parameter_handler();
}

void LineDetectionHoughPNode::declare_parameters() {
  this->declare_parameter<std::string>("topic.image_sub_topic");
  this->declare_parameter<std::string>("topic.line_segments_pub_topic");
  this->declare_parameter<std::string>("topic.color_overlay_pub_topic");
  this->declare_parameter<std::string>("topic.canny_debug_pub_topic");
  this->declare_parameter<std::string>("topic.canny_overlay_pub_topic");

  this->declare_parameter<int>("canny.low_threshold");
  this->declare_parameter<int>("canny.high_threshold");
  this->declare_parameter<int>("canny.aperture_size");
  this->declare_parameter<bool>("canny.L2_gradient");

  this->declare_parameter<double>("houghp.rho");
  this->declare_parameter<double>("houghp.theta");
  this->declare_parameter<int>("houghp.threshold");
  this->declare_parameter<double>("houghp.min_line_length");
  this->declare_parameter<double>("houghp.max_line_gap");

  this->declare_parameter<std::string>("mode");
}

void LineDetectionHoughPNode::setup_publishers_and_subscribers() {
  const std::string image_sub_topic =
      this->get_parameter("topic.image_sub_topic").as_string();
  const std::string line_segments_pub_topic =
      this->get_parameter("topic.line_segments_pub_topic").as_string();
  const std::string color_overlay_pub_topic =
      this->get_parameter("topic.color_overlay_pub_topic").as_string();
  const std::string canny_debug_pub_topic =
      this->get_parameter("topic.canny_debug_pub_topic").as_string();
  const std::string canny_overlay_pub_topic =
      this->get_parameter("topic.canny_overlay_pub_topic").as_string();

  const auto qos_profile = vortex::utils::qos_profiles::sensor_data_profile(1);

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

  canny_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      canny_debug_pub_topic, qos_profile);

  canny_overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      canny_overlay_pub_topic, qos_profile);
}

void LineDetectionHoughPNode::set_detector() {
  CannyConfig edge_config;
  edge_config.low_threshold =
      this->get_parameter("canny.low_threshold").as_int();
  edge_config.high_threshold =
      this->get_parameter("canny.high_threshold").as_int();
  edge_config.aperture_size =
      this->get_parameter("canny.aperture_size").as_int();
  edge_config.L2_gradient = this->get_parameter("canny.L2_gradient").as_bool();

  HoughPConfig line_config;
  line_config.rho = this->get_parameter("houghp.rho").as_double();
  line_config.theta = this->get_parameter("houghp.theta").as_double();
  line_config.threshold = this->get_parameter("houghp.threshold").as_int();
  line_config.min_line_length =
      this->get_parameter("houghp.min_line_length").as_double();
  line_config.max_line_gap =
      this->get_parameter("houghp.max_line_gap").as_double();

  detector_ = std::make_unique<LineDetectorHoughP>(edge_config, line_config);
}

void LineDetectionHoughPNode::image_callback(
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

RCLCPP_COMPONENTS_REGISTER_NODE(LineDetectionHoughPNode)

}  // namespace vortex::line_detection
