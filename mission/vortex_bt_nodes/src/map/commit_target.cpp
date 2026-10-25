#include "vortex_bt_nodes/map/commit_target.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

namespace vortex_bt_nodes::map {

CommitTarget::CommitTarget(const std::string& name,
                           const BT::NodeConfig& config,
                           rclcpp::Node::SharedPtr node,
                           std::shared_ptr<const LandmarkMap> map)
    : BT::StatefulActionNode(name, config),
      node_(std::move(node)),
      map_(std::move(map)) {}

BT::PortsList CommitTarget::providedPorts() {
    return {BT::InputPort<std::string>("frame", "Target TF frame"),
            BT::InputPort<double>("stable_m",
                                  "Position stability radius [m], nonnegative"),
            BT::InputPort<double>(
                "stable_s", "Continuous stability duration [s], nonnegative"),
            BT::OutputPort<Pose>("pose", "Committed target pose in odom")};
}

BT::NodeStatus CommitTarget::onStart() {
    anchor_.reset();
    const auto frame = getInput<std::string>("frame");
    const auto stable_m = getInput<double>("stable_m");
    const auto stable_s = getInput<double>("stable_s");
    if (!frame || frame->empty() || !stable_m || !std::isfinite(*stable_m) ||
        *stable_m < 0.0 || !stable_s || !std::isfinite(*stable_s) ||
        *stable_s < 0.0) {
        spdlog::error(
            "[{}] frame must be nonempty and stable_m/stable_s finite and "
            "nonnegative",
            name());
        return BT::NodeStatus::FAILURE;
    }
    frame_ = map_->frame_name(*frame);
    stable_m_ = *stable_m;
    stable_s_ = *stable_s;
    last_tick_ = node_->now();
    return onRunning();
}

BT::NodeStatus CommitTarget::onRunning() {
    const auto pose = map_->lookup(map_->odom_frame(), frame_);
    const auto now = node_->now();
    if (now < last_tick_) {
        anchor_.reset();
    }
    last_tick_ = now;
    if (!pose) {
        anchor_.reset();
        return BT::NodeStatus::RUNNING;
    }
    if (!anchor_ ||
        std::hypot(pose->position.x - anchor_->position.x,
                   pose->position.y - anchor_->position.y,
                   pose->position.z - anchor_->position.z) > stable_m_) {
        anchor_ = *pose;
        stable_since_ = now;
    }
    if ((now - stable_since_).seconds() < stable_s_) {
        return BT::NodeStatus::RUNNING;
    }
    const auto output = setOutput("pose", *pose);
    if (!output) {
        spdlog::error("[{}] cannot write committed pose: {}", name(),
                      output.error());
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

void CommitTarget::onHalted() {
    anchor_.reset();
}

}  // namespace vortex_bt_nodes::map
