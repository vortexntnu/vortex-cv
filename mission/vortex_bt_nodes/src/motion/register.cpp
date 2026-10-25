#include "vortex_bt_nodes/motion/register.hpp"
#include "vortex_bt_nodes/motion/search.hpp"

namespace vortex_bt_nodes::motion {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    const std::shared_ptr<const map::LandmarkMap> map =
        std::make_shared<map::LandmarkMap>(node);
    factory.registerNodeType<Search>("Search", node, map);
}

}  // namespace vortex_bt_nodes::motion
