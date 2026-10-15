#include "vortex_bt_nodes/mission/register.hpp"

#include "vortex_bt_nodes/mission/wait.hpp"

#include "vortex_bt_nodes/mission/Load_mission_config.hpp"

namespace vortex_bt_nodes::mission {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<Wait>("Wait", node);
    factory.registerNodeType<LoadMissionConfig>("LoadMissionConfig",node);
}

}  // namespace vortex_bt_nodes::mission
