#include <rclcpp/rclcpp.hpp>

#include "pipeline_polyline_follower/follower_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PipelinePolylineFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
