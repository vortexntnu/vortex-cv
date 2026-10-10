#include "vortex_bt_nodes/mission/register.hpp"

namespace vortex_bt_nodes::mission {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& /*node*/) {
    // factory.registerNodeType<MyNode>("MyNode", node);
}

}  // namespace vortex_bt_nodes::mission
