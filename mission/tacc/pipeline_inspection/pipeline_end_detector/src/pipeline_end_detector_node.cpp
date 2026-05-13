#include "pipeline_end_detector/pipeline_end_detector_node.hpp"

#include "vortex/utils/ros/qos_profiles.hpp"

namespace pipeline_end_detector {

PipelineEndDetectorNode::PipelineEndDetectorNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("pipeline_end_detector_node", options) {
    declare_parameters();

    detection_threshold_ =
        static_cast<int>(get_parameter("detection_threshold").as_int());

    setup_pubsub();

    RCLCPP_INFO(
        get_logger(),
        "PipelineEndDetectorNode started. threshold=%d, awaiting activation on "
        "'%s'",
        detection_threshold_,
        get_parameter("topics.start_detection_service").as_string().c_str());
}

void PipelineEndDetectorNode::declare_parameters() {
    declare_parameter<std::string>("topics.detection");
    declare_parameter<std::string>("topics.end_of_pipeline_service");
    declare_parameter<std::string>("topics.start_detection_service");
    declare_parameter<int>("detection_threshold");
}

void PipelineEndDetectorNode::setup_pubsub() {
    const auto sensor_qos = vortex::utils::qos_profiles::reliable_profile(10);
    detection_sub_ = create_subscription<std_msgs::msg::UInt8>(
        get_parameter("topics.detection").as_string(), sensor_qos,
        std::bind(&PipelineEndDetectorNode::detection_callback, this,
                  std::placeholders::_1));

    end_of_pipeline_client_ = create_client<std_srvs::srv::Trigger>(
        get_parameter("topics.end_of_pipeline_service").as_string());

    start_detection_server_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("topics.start_detection_service").as_string(),
        std::bind(&PipelineEndDetectorNode::start_detection_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

void PipelineEndDetectorNode::start_detection_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    detection_active_ = true;
    response->success = true;
    response->message = "Pipeline end detection activated.";
    RCLCPP_INFO(get_logger(), "Pipeline following started — detection active.");
}

void PipelineEndDetectorNode::detection_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
    if (!detection_active_ || service_called_) {
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
                rclcpp::shutdown();
            } else {
                RCLCPP_ERROR(get_logger(),
                             "End-of-pipeline service call failed: %s",
                             response->message.c_str());
                service_called_ = false;
                consecutive_detections_ = 0;
            }
        });
}

}  // namespace pipeline_end_detector
