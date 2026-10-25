#include "vortex_bt_nodes/motion/register.hpp"
#include "vortex_bt_nodes/motion/move_relative_node.hpp"
namespace vortex_bt_nodes::motion {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<MoveRelative>("MoveRelative", node);
}

}  // namespace vortex_bt_nodes::motion
