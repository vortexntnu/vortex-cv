#ifndef VORTEX_BT_NODES__MAP__LANDMARK_KNOWN_HPP_
#define VORTEX_BT_NODES__MAP__LANDMARK_KNOWN_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <memory>
#include <string>
#include <utility>

#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief SUCCESS if the landmark (id, or the best of type/subtype) is in the
 * map, from the prior map or observed.
 */
class LandmarkKnown : public BT::ConditionNode {
   public:
    LandmarkKnown(const std::string& name,
                  const BT::NodeConfig& config,
                  std::shared_ptr<const LandmarkCache> cache)
        : BT::ConditionNode(name, config), cache_(std::move(cache)) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

   private:
    std::shared_ptr<const LandmarkCache> cache_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__LANDMARK_KNOWN_HPP_
