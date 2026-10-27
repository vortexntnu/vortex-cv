#include "board_detection_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace vortex::board_detection {

BoardDetectionNode::BoardDetectionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("board_detection_node", options) {
    declare_parameters();
    setup_publishers_and_subscribers();
}

void BoardDetectionNode::declare_parameters() {

}

void BoardDetectionNode::setup_publishers_and_subscribers() {

}

void BoardDetectionNode::image_callback() {

}