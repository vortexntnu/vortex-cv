#include <rclcpp/rclcpp.hpp>
#include "bearing_direction_server/acoustics_bearing_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<bearing_direction_server::AcousticsBearingNode>());
    rclcpp::shutdown();
    return 0;
}
