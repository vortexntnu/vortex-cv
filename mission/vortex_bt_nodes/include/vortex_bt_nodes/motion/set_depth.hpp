#ifndef VORTEX_BT_NODES__MOTION__SET_DEPTH_HPP_
#define VORTEX_BT_NODES__MOTION__SET_DEPTH_HPP_

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::motion {

class SetDepth : public NavAction {
   public:
    SetDepth(const std::string& name,
             const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__SET_DEPTH_HPP_