#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace pipeline_end_detector {

class PipelineEndDetectorNode : public rclcpp::Node {
   public:
    explicit PipelineEndDetectorNode(const rclcpp::NodeOptions& options);

   private:
    void declare_parameters();
    void setup_pubsub();
    void detection_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void call_end_of_pipeline_service();

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr detection_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_of_pipeline_client_;

    int consecutive_detections_{
        0};  // number of consecutive Class 1 detections so far
    int detection_threshold_;  // number of consecutive detections required to
                               // trigger the service call
    bool service_called_{
        false};  // prevents the service from being called more than once
};

}  // namespace pipeline_end_detector
