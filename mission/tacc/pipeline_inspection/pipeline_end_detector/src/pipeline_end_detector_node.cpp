#include "pipeline_end_detector/pipeline_end_detector_node.hpp"

#include "vortex/utils/ros/qos_profiles.hpp"

namespace pipeline_end_detector {

PipelineEndDetectorNode::PipelineEndDetectorNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("pipeline_end_detector_node", options) {
    declare_parameters();

    detection_threshold_ =
        static_cast<int>(get_parameter("detection_threshold").as_int());
    activation_delay_sec_ = get_parameter("activation_delay_sec").as_double();

    setup_pubsub();

    RCLCPP_INFO(
        get_logger(),
        "PipelineEndDetectorNode started. threshold=%d, activation_delay=%.1fs, "
        "awaiting activation on '%s'",
        detection_threshold_, activation_delay_sec_,
        get_parameter("topics.start_detection_service").as_string().c_str());
}

void PipelineEndDetectorNode::declare_parameters() {
    declare_parameter<std::string>("topics.detection");
    declare_parameter<std::string>("topics.end_of_pipeline_service");
    declare_parameter<std::string>("topics.start_detection_service");
    declare_parameter<int>("detection_threshold");
    declare_parameter<double>("activation_delay_sec", 0.0);
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
        std::bind(
            &PipelineEndDetectorNode::start_end_pipeline_detection_callback,
            this, std::placeholders::_1, std::placeholders::_2));
}

void PipelineEndDetectorNode::start_end_pipeline_detection_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;

    if (activation_delay_sec_ <= 0.0) {
        activate_detection();
        response->message = "Pipeline end detection activated.";
        return;
    }

    // Acknowledge immediately so the FSM can proceed straight into pipeline
    // following; arm a one-shot timer that activates detection after the delay.
    response->message = "Pipeline end detection scheduled.";
    RCLCPP_INFO(get_logger(),
                "Pipeline following started — detection will activate in %.1fs.",
                activation_delay_sec_);
    activation_timer_ = create_wall_timer(
        std::chrono::duration<double>(activation_delay_sec_), [this]() {
            activation_timer_->cancel();  // one-shot
            activate_detection();
        });
}

void PipelineEndDetectorNode::activate_detection() {
    detection_active_ = true;
    RCLCPP_INFO(get_logger(), "Pipeline end detection active.");
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
