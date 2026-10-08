#include "vortex_bt_nodes/actuators/drop_marker.hpp"

#include <spdlog/spdlog.h>

#include <vortex/utils/ros/qos_profiles.hpp>

namespace vortex_bt_nodes::actuators {

BT::PortsList DropMarker::providedPorts() {
    return {
        BT::InputPort<int>("index", "0 or 1"),
        BT::InputPort<std::string>("topic", "dropper/drop", "std_msgs/Int8"),
        BT::InputPort<double>("settle_s", 2.0, "Wait after it")};
}

BT::NodeStatus DropMarker::onStart() {
    const auto value = getInput<int>("index");
    if (!value || (*value != 0 && *value != 1)) {
        spdlog::warn("[{}] index must be 0 or 1", name());
        return BT::NodeStatus::FAILURE;
    }
    const auto data = static_cast<std::int8_t>(*value);
    const std::string topic =
        getInput<std::string>("topic").value_or("dropper/drop");
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

BT::NodeStatus DropMarker::onRunning() {
    return (node_->now() - start_).seconds() >=
                   getInput<double>("settle_s").value_or(2.0)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::RUNNING;
}

}  // namespace vortex_bt_nodes::actuators
