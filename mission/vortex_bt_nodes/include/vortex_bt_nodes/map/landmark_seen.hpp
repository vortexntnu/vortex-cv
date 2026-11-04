#ifndef VORTEX_BT_NODES__MAP__LANDMARK_SEEN_HPP_
#define VORTEX_BT_NODES__MAP__LANDMARK_SEEN_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <memory>
#include <string>
#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

/** @brief SUCCESS if the landmark was detected in the last within_s seconds. */
class LandmarkSeen : public BT::ConditionNode {
   public:
    LandmarkSeen(const std::string& name,
                 const BT::NodeConfig& config,
                 std::shared_ptr<const LandmarkMap> map);

    static BT::PortsList providedPorts();

   private:
    std::shared_ptr<const LandmarkMap> map_;
    BT::NodeStatus tick() override;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__LANDMARK_SEEN_HPP_
