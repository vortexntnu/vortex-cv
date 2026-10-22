#include "vortex_bt_nodes/motion/register.hpp"

#include "vortex_bt_nodes/motion/set_depth.hpp"


namespace vortex_bt_nodes::motion {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<SetDepth>("SetDepth", node);
}

}  // namespace vortex_bt_nodes::motion
