#ifndef VORTEX_BT_NODES__MOTION__SURFACE_HPP_
#define VORTEX_BT_NODES__MOTION__SURFACE_HPP_

#include <string>
#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::motion {

class Surface : public NavAction {
   public:
    Surface(const std::string& name,
            const BT::NodeConfig& config,
            rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__SURFACE_HPP_
