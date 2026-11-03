#include "board_detection_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace vortex::board_detection {

BoardDetectionNode::BoardDetectionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("board_detection_node", options) {
    declare_parameters();
    setup_publishers_and_subscribers();
}

void BoardDetectionNode::declare_parameters() {
    this->declare_parameter<std::string>("topic.image_sub_topic");

}

void BoardDetectionNode::setup_publishers_and_subscribers() {
    const std::string mask_sub_topic =
        this->get_parameter("topic.image_sub_topic").as_string();

    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    mask_sub_topic, qos_profile,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        this->image_callback(msg);
    });


}

void BoardDetectionNode::board_callback(
    //board mask
    //depth image
    //drone pos synced
) { 
    //First off: Run checks (if img is treatable etc.)
    //Step 0: Define board 1 or board 2
    //Step 1: Convert ROS image message to OpenCV image
    //Step 2: Turn stereo-image to point cloud
    //Step 3: RANSAC to find the board plane
    //Step 4: Use plane + mask to find board center (+ corners)
    //Step 5: Transfer board pose to world frame
    //Step 6: publish board pose
}

void BoardDetectionNode::circle_callback(
    // Circle mask
    // Depth image
    // Board_pose from board_callback
    // board_type
) {
    //First off: Run checks (if img is treatable etc.)
    //Step 1: Find center of circle ("draw line from camera to plane")
    //Step 2: Find distance from circle center to board edges - 
        //Based on edge distance, define circle category
    //Publish circle position + category
}