#include "bearing_direction_server/pipeline_bearing_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bearing_direction_server::PipelineBearingNode>());
    rclcpp::shutdown();
    return 0;
}
