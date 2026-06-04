#include "bearing_direction_server/pipeline_bearing_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(
        std::make_shared<bearing_direction_server::PipelineBearingNode>());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
