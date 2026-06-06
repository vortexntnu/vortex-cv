#include <pipeline_intersection_following/line_filtering_ros.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFilteringNode>());
    rclcpp::shutdown();
    return 0;
}
