#include "vortex_bt_nodes/map/register.hpp"

#include <memory>

#include "vortex_bt_nodes/map/landmark_confirmed.hpp"
#include "vortex_bt_nodes/map/go_to_frame_node.hpp"
#include "vortex_bt_nodes/map/landmark_known.hpp"
#include "vortex_bt_nodes/map/landmark_map.hpp"
#include "vortex_bt_nodes/map/look_at_frame.hpp"
#include "vortex_bt_nodes/map/turn.hpp"

namespace vortex_bt_nodes::map {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    // One map for all map nodes. Pass it to the nodes that need it.
    const std::shared_ptr<const LandmarkMap> map =
        std::make_shared<LandmarkMap>(node);
    factory.registerNodeType<LandmarkConfirmed>("LandmarkConfirmed", map);
    factory.registerNodeType<LookAtFrame>("LookAtFrame", node, map);
    factory.registerNodeType<Turn>("Turn", node, map);

    factory.registerNodeType<GoToFrame>("GoToFrame", node, map);
    factory.registerNodeType<LandmarkKnown>("LandmarkKnown", map);
}

}  // namespace vortex_bt_nodes::map
