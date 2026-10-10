#include "vortex_bt_nodes/approach/register.hpp"

namespace vortex_bt_nodes::approach {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& /*node*/) {
    // One line per node, e.g.
    //   factory.registerNodeType<MyNode>("MyNode");
    // or, for a node whose constructor also takes the ROS node:
    //   factory.registerNodeType<MyRosNode>("MyRosNode", node);
    // Each node: include/vortex_bt_nodes/approach/<node>.hpp and
    // src/approach/<node>.cpp.
}

}  // namespace vortex_bt_nodes::approach
