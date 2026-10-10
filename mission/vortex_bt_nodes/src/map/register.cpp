#include "vortex_bt_nodes/map/register.hpp"

#include <memory>

#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

void register_nodes(BT::BehaviorTreeFactory& /*factory*/,
                    const rclcpp::Node::SharedPtr& node) {
    // One cache for all map nodes. Pass it to the nodes that need it.
    [[maybe_unused]] const std::shared_ptr<const LandmarkCache> cache =
        std::make_shared<LandmarkCache>(node);
    // factory.registerNodeType<MyNode>("MyNode", node, cache);
}

}  // namespace vortex_bt_nodes::map
