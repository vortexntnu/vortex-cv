#ifndef VORTEX_BT_NODES__MAP__LANDMARK_CONFIRMED_HPP_
#define VORTEX_BT_NODES__MAP__LANDMARK_CONFIRMED_HPP_

#include <behaviortree_cpp/condition_node.h>

#include <memory>
#include <string>
#include <utility>

#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief SUCCESS if the landmark has been observed and its horizontal position
 * std (relative to the vehicle) is below max_sigma_xy.
 */
class LandmarkConfirmed : public BT::ConditionNode {
   public:
    LandmarkConfirmed(const std::string& name,
                      const BT::NodeConfig& config,
                      std::shared_ptr<const LandmarkCache> cache)
        : BT::ConditionNode(name, config), cache_(std::move(cache)) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

   private:
    std::shared_ptr<const LandmarkCache> cache_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__LANDMARK_CONFIRMED_HPP_
