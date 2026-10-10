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
 * @brief Waits until a TF frame stands still, then writes its pose (in odom)
 * to the blackboard: the target is committed, and GoToPose drives to it
 * without following the map any more.
 *
 * A frame from the map moves while detections correct it. Close to an
 * object, or beside it, the detections get worse or stop, so the precise
 * part of a task (through a gap, in front of an opening) is driven on a
 * target frozen at a good viewpoint. RUNNING while the frame is missing or
 * has moved more than stable_m in the last stable_s; SUCCESS when it has not.
 * Put a Timeout around it: a frame that never settles must not stop the
 * task.
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
    // Where the frame was when it last moved more than stable_m, and when.
    std::optional<Pose> anchor_;
    std::chrono::steady_clock::time_point anchor_time_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__COMMIT_TARGET_HPP_
