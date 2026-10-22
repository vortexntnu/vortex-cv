#ifndef VORTEX_BT_NODES__MAP__LANDMARK_KNOWN_HPP_
#define VORTEX_BT_NODES__MAP__LANDMARK_KNOWN_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

class LandmarkKnown : public BT::ConditionNode {
   public:
    LandmarkKnown(const std::string& name,
                  const BT::NodeConfig& config,
                  std::shared_ptr<const LandmarkMap> map);

    static BT::PortsList providedPorts();

   private:
    std::shared_ptr<const LandmarkMap> map_;
    BT::NodeStatus tick() override;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__LANDMARK_KNOWN_HPP_
