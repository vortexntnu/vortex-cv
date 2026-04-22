#include "pipeline_end_detector/pipeline_end_detector_node.hpp"

namespace pipeline_end_detector {

PipelineEndDetectorNode::PipelineEndDetectorNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("pipeline_end_detector_node", options) {
    declare_parameters();

    detection_threshold_ =
        static_cast<int>(get_parameter("detection_threshold").as_int());

    setup_pubsub();

    RCLCPP_INFO(get_logger(),
                "PipelineEndDetectorNode started. threshold=%d, service='%s'",
                detection_threshold_,
                get_parameter("topics.end_of_pipeline_service").as_string().c_str());
}

void PipelineEndDetectorNode::declare_parameters() {
    declare_parameter<std::string>("topics.detection",
                                  "end_of_pipeline/detection");
    declare_parameter<std::string>("topics.end_of_pipeline_service",
                                  "pipeline_inspection_fsm/pipeline_finished");
    declare_parameter<int>("detection_threshold", 10);
}

void PipelineEndDetectorNode::setup_pubsub() {
    detection_sub_ = create_subscription<std_msgs::msg::UInt8>(
        get_parameter("topics.detection").as_string(),
        rclcpp::QoS(10),
        std::bind(&PipelineEndDetectorNode::detection_callback, this,
                  std::placeholders::_1));

    end_of_pipeline_client_ = create_client<std_srvs::srv::Trigger>(
        get_parameter("topics.end_of_pipeline_service").as_string());
}

void PipelineEndDetectorNode::detection_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
    if (service_called_) {
        return;
    }

    if (msg->data > 0) {
        ++consecutive_detections_;
        RCLCPP_DEBUG(get_logger(), "Consecutive detections: %d / %d",
                     consecutive_detections_, detection_threshold_);
    } else {
        if (consecutive_detections_ > 0) {
            RCLCPP_DEBUG(get_logger(),
                         "Detection streak broken, resetting counter.");
        }
        consecutive_detections_ = 0;
    }

    if (consecutive_detections_ >= detection_threshold_) {
        call_end_of_pipeline_service();
    }
}

void PipelineEndDetectorNode::call_end_of_pipeline_service() {
    if (!end_of_pipeline_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(),
                    "End-of-pipeline service not available, skipping call.");
        return;
    }

    service_called_ = true;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = end_of_pipeline_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
            const auto& response = result.get();
            if (response->success) {
                RCLCPP_INFO(get_logger(),
                            "End-of-pipeline service call succeeded: %s",
                            response->message.c_str());
            } else {
                RCLCPP_ERROR(get_logger(),
                             "End-of-pipeline service call failed: %s",
                             response->message.c_str());
                // Allow retrying on next detection streak
                service_called_ = false;
                consecutive_detections_ = 0;
            }
        });
}

}  // namespace pipeline_end_detector
