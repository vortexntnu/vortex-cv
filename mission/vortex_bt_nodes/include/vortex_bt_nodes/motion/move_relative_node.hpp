#ifndef VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_NODE_HPP_
#define VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vortex_bt_nodes/common/nav_action.hpp>
#include <vortex_bt_nodes/map/landmark_map.hpp>

namespace vortex_bt_nodes::motion {

class MoveRelative : NavAction {
   public:
    MoveRelative(const std::string& name,
                 const BT::NodeConfig& config,
                 const rclcpp::Node::SharedPtr& node,
                 const std::shared_ptr<const map::LandmarkMap> map);

    static BT::PortsList providedPorts();

   private:
    std::optional<Goal> make_goal() override;
    std::optional<Goal> update_goal() override;

    const std::shared_ptr<const map::LandmarkMap> map_;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_NODE_HPP_
