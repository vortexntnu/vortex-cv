#include "robosub_mission/register_nodes.hpp"

namespace robosub_mission {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& /*node*/) {
    // Custom nodes are registered here, one line each, for example
    //   factory.registerNodeType<ApproachLandmark>("ApproachLandmark", node);
    // Each node gets its own header in include/robosub_mission/nodes/ and
    // source in src/nodes/.
}

}  // namespace robosub_mission
