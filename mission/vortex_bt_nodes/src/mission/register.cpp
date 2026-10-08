#include "vortex_bt_nodes/mission/register.hpp"

#include "vortex_bt_nodes/mission/load_mission_config.hpp"
#include "vortex_bt_nodes/mission/log.hpp"
#include "vortex_bt_nodes/mission/start_run.hpp"
#include "vortex_bt_nodes/mission/wait_for_start.hpp"

namespace vortex_bt_nodes::mission {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<WaitForStart>("WaitForStart", node);
    factory.registerNodeType<StartRun>("StartRun", node);
    factory.registerNodeType<LoadMissionConfig>("LoadMissionConfig");
    factory.registerNodeType<Log>("Log");
}

}  // namespace vortex_bt_nodes::mission
