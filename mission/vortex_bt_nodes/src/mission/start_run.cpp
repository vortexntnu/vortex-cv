#include "vortex_bt_nodes/mission/start_run.hpp"

#include <spdlog/spdlog.h>

#include <chrono>

#include <vortex/utils/ros/qos_profiles.hpp>

namespace vortex_bt_nodes::mission {

namespace {
constexpr double kResendS = 2.0;
}  // namespace

StartRun::StartRun(const std::string& name,
                   const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(std::move(node)) {
    wipe_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
        "mission/wipe", vortex::utils::qos_profiles::reliable_profile(1));
}

BT::PortsList StartRun::providedPorts() {
    return {BT::InputPort<double>("coin_flip_deg", 0.0,
                                  "Start heading relative to the prior map's"),
            BT::InputPort<std::string>("slam_node", "landmark_slam_node",
                                       "landmark_slam node name"),
            BT::InputPort<double>("timeout_s", 10.0, "Wait for landmark_slam")};
}

BT::NodeStatus StartRun::onStart() {
    wipe_pub_->publish(std_msgs::msg::Empty());
    if (!params_) {
        params_ = std::make_shared<rclcpp::AsyncParametersClient>(
            node_,
            getInput<std::string>("slam_node").value_or("landmark_slam_node"));
    }
    pending_.reset();
    start_ = node_->now();
    return onRunning();
}

BT::NodeStatus StartRun::onRunning() {
    const double coin_flip = getInput<double>("coin_flip_deg").value_or(0.0);
    const double elapsed = (node_->now() - start_).seconds();
    if (elapsed > getInput<double>("timeout_s").value_or(10.0)) {
        spdlog::error("[{}] landmark_slam did not answer", name());
        return BT::NodeStatus::FAILURE;
    }
    // A reply can be lost while the service is still being discovered:
    // setting the same value again is harmless, so ask again.
    const bool resend =
        pending_ && (node_->now() - sent_).seconds() > kResendS;
    if ((!pending_ || resend) && params_->service_is_ready()) {
        pending_ = params_->set_parameters(
            {rclcpp::Parameter("start_yaw_offset_deg", coin_flip)});
        sent_ = node_->now();
        return BT::NodeStatus::RUNNING;
    }
    if (!pending_ || pending_->wait_for(std::chrono::seconds(0)) !=
                         std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }
    const auto results = pending_->get();
    if (results.empty() || !results.front().successful) {
        spdlog::error("[{}] landmark_slam rejected the coin flip: {}", name(),
                      results.empty() ? "" : results.front().reason);
        return BT::NodeStatus::FAILURE;
    }
    spdlog::info("[{}] run started, coin flip {:+.0f} deg", name(), coin_flip);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace vortex_bt_nodes::mission
