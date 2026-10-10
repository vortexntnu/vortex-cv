#include "vortex_bt_nodes/actuators/register.hpp"

namespace vortex_bt_nodes::actuators {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& /*node*/) {
    // factory.registerNodeType<MyNode>("MyNode", node);
}

}  // namespace vortex_bt_nodes::actuators
