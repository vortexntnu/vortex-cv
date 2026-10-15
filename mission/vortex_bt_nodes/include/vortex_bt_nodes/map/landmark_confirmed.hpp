#ifndef VORTEX_BT_NODES__MAP__LANDMARK_CONFIRMED_HPP_
#define VORTEX_BT_NODES__MAP__LANDMARK_CONFIRMED_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <memory>
#include <string>

#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief SUCCESS if the landmark has been obmax_sigma_xyserved and its horizontal std
 * is below max_sigma_xy.
 */
class LandmarkConfirmed : public BT::ConditionNode {
   public:
    LandmarkConfirmed(const std::string& name,
                      const BT::NodeConfig& config,
                      std::shared_ptr<const LandmarkMap> map);

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus tick() override;

    std::shared_ptr<const LandmarkMap> map_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__LANDMARK_CONFIRMED_HPP_
