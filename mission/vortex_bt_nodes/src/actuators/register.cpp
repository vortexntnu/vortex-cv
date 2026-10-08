#include "vortex_bt_nodes/actuators/register.hpp"

#include "vortex_bt_nodes/actuators/drop_marker.hpp"
#include "vortex_bt_nodes/actuators/fire_torpedo.hpp"
#include "vortex_bt_nodes/actuators/set_gripper.hpp"

namespace vortex_bt_nodes::actuators {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<FireTorpedo>("FireTorpedo", node);
    factory.registerNodeType<DropMarker>("DropMarker", node);
    factory.registerNodeType<SetGripper>("SetGripper", node);
}

}  // namespace vortex_bt_nodes::actuators
