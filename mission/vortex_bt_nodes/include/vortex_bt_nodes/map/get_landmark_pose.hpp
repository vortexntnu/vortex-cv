#ifndef VORTEX_BT_NODES__MAP__GET_LANDMARK_POSE_HPP_
#define VORTEX_BT_NODES__MAP__GET_LANDMARK_POSE_HPP_

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>
#include <utility>

#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Writes the landmark pose (map frame) to pose. FAILURE if unknown.
 */
class GetLandmarkPose : public BT::SyncActionNode {
   public:
    GetLandmarkPose(const std::string& name,
                    const BT::NodeConfig& config,
                    std::shared_ptr<const LandmarkCache> cache)
        : BT::SyncActionNode(name, config), cache_(std::move(cache)) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

   private:
    std::shared_ptr<const LandmarkCache> cache_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__GET_LANDMARK_POSE_HPP_
