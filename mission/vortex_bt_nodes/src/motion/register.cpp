#include "vortex_bt_nodes/motion/register.hpp"

#include "vortex_bt_nodes/motion/move_relative.hpp"
#include "vortex_bt_nodes/motion/search.hpp"
#include "vortex_bt_nodes/motion/set_depth.hpp"
#include "vortex_bt_nodes/motion/surface.hpp"

namespace vortex_bt_nodes::motion {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<SetDepth>("SetDepth", node);
    factory.registerNodeType<Surface>("Surface", node);
    factory.registerNodeType<MoveRelative>("MoveRelative", node);
    factory.registerNodeType<Search>("Search", node);
}

}  // namespace vortex_bt_nodes::motion
