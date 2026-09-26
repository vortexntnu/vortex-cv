#include "robosub_mission/nodes/mission/register.hpp"

namespace robosub_mission::nodes::mission {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& /*node*/) {
    // One line per node, e.g.
    //   factory.registerNodeType<MyNode>("MyNode");
    // or, for a node whose constructor also takes the ROS node:
    //   factory.registerNodeType<MyRosNode>("MyRosNode", node);
    // Each node: include/robosub_mission/nodes/mission/<node>.hpp and
    // src/nodes/mission/<node>.cpp.
}

}  // namespace robosub_mission::nodes::mission
