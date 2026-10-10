#include "vortex_bt_nodes/motion/register.hpp"

namespace vortex_bt_nodes::motion {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& /*node*/) {
    // factory.registerNodeType<MyNode>("MyNode", node);
}

}  // namespace vortex_bt_nodes::motion
