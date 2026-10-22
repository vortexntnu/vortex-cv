#include "bearing_localization/ros/bearing_localization_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<bearing_localization::BearingLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
