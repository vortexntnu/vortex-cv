#include "vortex_bt_nodes/mission/register.hpp"

#include "vortex_bt_nodes/mission/log.hpp"
#include "vortex_bt_nodes/mission/wait.hpp"
#include "vortex_bt_nodes/mission/wait_for_start.hpp"

namespace vortex_bt_nodes::mission {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<Wait>("Wait", node);
    factory.registerNodeType<Log>("Log");
    factory.registerNodeType<WaitForStart>("WaitForStart", node);
}

}  // namespace vortex_bt_nodes::mission
