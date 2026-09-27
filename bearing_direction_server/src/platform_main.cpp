#include <rclcpp/rclcpp.hpp>
#include "bearing_direction_server/platform_bearing_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<bearing_direction_server::PlatformBearingNode>());
    rclcpp::shutdown();
    return 0;
}
