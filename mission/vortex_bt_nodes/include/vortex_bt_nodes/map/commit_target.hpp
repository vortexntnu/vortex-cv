#ifndef VORTEX_BT_NODES__MAP__COMMIT_TARGET_HPP_
#define VORTEX_BT_NODES__MAP__COMMIT_TARGET_HPP_

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Waits until a TF frame has moved less than stable_m for stable_s,
 * then writes its pose in odom to the blackboard for GoToPose.
 *
 * Used before driving close to or past an object, where the detections get
 * worse and the frame should no longer be followed. Wrap it in a Timeout.
 */
class CommitTarget : public BT::StatefulActionNode {
   public:
    CommitTarget(const std::string& name,
                 const BT::NodeConfig& config,
                 std::shared_ptr<const LandmarkCache> cache)
        : BT::StatefulActionNode(name, config), cache_(std::move(cache)) {}

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

    std::shared_ptr<const LandmarkCache> cache_;
    std::optional<Pose> anchor_;
    std::chrono::steady_clock::time_point anchor_time_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__COMMIT_TARGET_HPP_
