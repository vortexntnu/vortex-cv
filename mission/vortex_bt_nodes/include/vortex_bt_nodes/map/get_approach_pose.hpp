#ifndef VORTEX_BT_NODES__MAP__GET_APPROACH_POSE_HPP_
#define VORTEX_BT_NODES__MAP__GET_APPROACH_POSE_HPP_

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>
#include <utility>

#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Writes a pose at offset from the landmark (landmark frame), facing it,
 * in odom (drift-corrected), on the symmetric side closest to the vehicle.
 * FAILURE if the landmark or the vehicle pose is unknown.
 */
class GetApproachPose : public BT::SyncActionNode {
   public:
    GetApproachPose(const std::string& name,
                    const BT::NodeConfig& config,
                    std::shared_ptr<const LandmarkCache> cache)
        : BT::SyncActionNode(name, config), cache_(std::move(cache)) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

   private:
    std::shared_ptr<const LandmarkCache> cache_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__GET_APPROACH_POSE_HPP_
