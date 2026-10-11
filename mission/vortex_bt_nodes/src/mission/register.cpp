#include "vortex_bt_nodes/mission/register.hpp"

#include "vortex_bt_nodes/mission/wait.hpp"

namespace vortex_bt_nodes::mission {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<Wait>("Wait", node);
}

}  // namespace vortex_bt_nodes::mission
