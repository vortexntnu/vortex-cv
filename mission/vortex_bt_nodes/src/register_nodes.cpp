#include "vortex_bt_nodes/register_nodes.hpp"
#include "vortex_bt_nodes/actuators/register.hpp"
#include "vortex_bt_nodes/approach/register.hpp"
#include "vortex_bt_nodes/map/register.hpp"
#include "vortex_bt_nodes/mission/register.hpp"
#include "vortex_bt_nodes/motion/register.hpp"

namespace vortex_bt_nodes {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    motion::register_nodes(factory, node);
    map::register_nodes(factory, node);
    approach::register_nodes(factory, node);
    actuators::register_nodes(factory, node);
    mission::register_nodes(factory, node);
}

}  // namespace vortex_bt_nodes
