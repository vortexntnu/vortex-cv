#include "vortex_bt_nodes/map/register.hpp"

#include <memory>

#include "vortex_bt_nodes/map/landmark_confirmed.hpp"
#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    // One map for all map nodes. Pass it to the nodes that need it.
    const std::shared_ptr<const LandmarkMap> map =
        std::make_shared<LandmarkMap>(node);
    factory.registerNodeType<LandmarkConfirmed>("LandmarkConfirmed", map);
}

}  // namespace vortex_bt_nodes::map
