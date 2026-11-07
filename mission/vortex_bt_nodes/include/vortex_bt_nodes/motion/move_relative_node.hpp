#ifndef VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_NODE_HPP_
#define VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>

#include <cstdint>
#include <optional>
#include <string>

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::motion {

/**
 * @brief Moves by an offset from where the vehicle is when the node starts.
 *
 * frame BODY: the offset is along the vehicle's own axes.
 * frame WORLD: the offset is along the odom axes, heading ignored.
 */
class MoveRelative : public NavAction {
   public:
    MoveRelative(const std::string& name,
                 const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_NODE_HPP_
