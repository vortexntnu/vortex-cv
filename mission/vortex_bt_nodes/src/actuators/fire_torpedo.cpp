#include "vortex_bt_nodes/actuators/fire_torpedo.hpp"

#include <spdlog/spdlog.h>

#include <vortex/utils/ros/qos_profiles.hpp>

namespace vortex_bt_nodes::actuators {

BT::PortsList FireTorpedo::providedPorts() {
    return {
        BT::InputPort<std::string>("side", "left or right"),
        BT::InputPort<std::string>("topic", "torpedo/fire", "std_msgs/Int8"),
        BT::InputPort<double>("settle_s", 1.0, "Wait after it")};
}

BT::NodeStatus FireTorpedo::onStart() {
    const auto value = getInput<std::string>("side");
    if (!value || (*value != "left" && *value != "right")) {
        spdlog::warn("[{}] side must be left or right", name());
        return BT::NodeStatus::FAILURE;
    }
    const std::int8_t data = *value == "left" ? 0 : 1;
    const std::string topic =
        getInput<std::string>("topic").value_or("torpedo/fire");
    if (!pub_ || topic != topic_) {
        pub_ = node_->create_publisher<std_msgs::msg::Int8>(
            topic, vortex::utils::qos_profiles::reliable_profile(1));
        topic_ = topic;
    }
    std_msgs::msg::Int8 msg;
    msg.data = data;
    pub_->publish(msg);
    spdlog::info("[{}] {} -> {}", name(), static_cast<int>(data), topic);
    start_ = node_->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FireTorpedo::onRunning() {
    return (node_->now() - start_).seconds() >=
                   getInput<double>("settle_s").value_or(1.0)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::RUNNING;
}

}  // namespace vortex_bt_nodes::actuators
