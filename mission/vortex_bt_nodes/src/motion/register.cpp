#include "vortex_bt_nodes/motion/register.hpp"

namespace vortex_bt_nodes::motion {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& /*node*/) {
    // One line per node, e.g.
    //   factory.registerNodeType<MyNode>("MyNode");
    // or, for a node whose constructor also takes the ROS node:
    //   factory.registerNodeType<MyRosNode>("MyRosNode", node);
    // Each node: include/vortex_bt_nodes/motion/<node>.hpp and
    // src/motion/<node>.cpp.
}

}  // namespace vortex_bt_nodes::motion
