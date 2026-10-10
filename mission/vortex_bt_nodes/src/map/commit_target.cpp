#include "vortex_bt_nodes/map/commit_target.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

namespace vortex_bt_nodes::map {

BT::PortsList CommitTarget::providedPorts() {
    return {BT::InputPort<std::string>("frame", "TF frame to commit to"),
            BT::InputPort<double>("stable_m", 0.05,
                                  "The frame stands still when it moved "
                                  "less than this [m] ..."),
            BT::InputPort<double>("stable_s", 2.0, "... for this long [s]"),
            BT::OutputPort<Pose>("pose", "The frame's pose in odom")};
}

BT::NodeStatus CommitTarget::onStart() {
    anchor_.reset();
    return onRunning();
}

BT::NodeStatus CommitTarget::onRunning() {
    const auto frame = getInput<std::string>("frame");
    if (!frame) {
        spdlog::warn("[{}] no frame", name());
        return BT::NodeStatus::FAILURE;
    }
    const auto pose =
        cache_->lookup(cache_->odom_frame(), cache_->frame_name(*frame));
    if (!pose) {
        anchor_.reset();  // not in the map (yet)
        return BT::NodeStatus::RUNNING;
    }
    const auto now = std::chrono::steady_clock::now();
    const double stable_m = getInput<double>("stable_m").value_or(0.05);
    const double stable_s = getInput<double>("stable_s").value_or(2.0);
    if (!anchor_ ||
        std::hypot(pose->position.x - anchor_->position.x,
                   pose->position.y - anchor_->position.y,
                   pose->position.z - anchor_->position.z) > stable_m) {
        anchor_ = *pose;
        anchor_time_ = now;
        return BT::NodeStatus::RUNNING;
    }
    if (std::chrono::duration<double>(now - anchor_time_).count() < stable_s) {
        return BT::NodeStatus::RUNNING;
    }
    setOutput("pose", *pose);
    spdlog::info("[{}] committed to '{}' at [{:.2f}, {:.2f}, {:.2f}] (odom)",
                 name(), *frame, pose->position.x, pose->position.y,
                 pose->position.z);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace vortex_bt_nodes::map
