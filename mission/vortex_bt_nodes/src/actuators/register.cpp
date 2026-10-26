#include "vortex_bt_nodes/actuators/register.hpp"
#include "vortex_bt_nodes/actuators/fire_torpedo_node.hpp"

namespace vortex_bt_nodes::actuators {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<FireTorpedo>("FireTorpedo", node);
}

}  // namespace vortex_bt_nodes::actuators
