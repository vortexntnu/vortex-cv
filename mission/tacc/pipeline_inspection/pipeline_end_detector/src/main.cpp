#include <rclcpp/rclcpp.hpp>
#include "pipeline_end_detector/pipeline_end_detector_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<pipeline_end_detector::PipelineEndDetectorNode>(
            rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
