#include <rclcpp/rclcpp.hpp>

#include "bearing_direction_server/test_acoustics_direction_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<bearing_direction_server::TestAcousticsDirectionNode>());
    rclcpp::shutdown();
    return 0;
}
