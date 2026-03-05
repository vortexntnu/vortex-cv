#ifndef LINE_DETECTION_RANSAC__ROS__LINE_DETECTION_RANSAC_ROS_HPP_
#define LINE_DETECTION_RANSAC__ROS__LINE_DETECTION_RANSAC_ROS_HPP_

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include "line_detection_ransac/lib/line_detection_ransac.hpp"

namespace vortex::line_detection {

class LineDetectionRansacNode : public rclcpp::Node {
 public:
  explicit LineDetectionRansacNode(const rclcpp::NodeOptions& options);

  ~LineDetectionRansacNode() override = default;

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<vortex_msgs::msg::LineSegment2DArray>::SharedPtr
      line_segments_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_overlay_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr boundary_debug_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr boundary_overlay_pub_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void declare_parameters();

  void set_detector();

  void setup_publishers_and_subscribers();

  void set_mode();

  void mode_conditional_publishing(const Result& result,
                                   const std_msgs::msg::Header& header);

  std::unique_ptr<LineDetectorRansac> detector_{};

  DetectorMode mode_{DetectorMode::standard};
};

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_RANSAC__ROS__LINE_DETECTION_RANSAC_ROS_HPP_
