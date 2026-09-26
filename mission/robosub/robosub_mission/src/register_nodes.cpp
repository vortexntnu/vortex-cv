#include "robosub_mission/register_nodes.hpp"
#include "robosub_mission/nodes/actuators/register.hpp"
#include "robosub_mission/nodes/approach/register.hpp"
#include "robosub_mission/nodes/map/register.hpp"
#include "robosub_mission/nodes/mission/register.hpp"
#include "robosub_mission/nodes/motion/register.hpp"

namespace robosub_mission {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    nodes::motion::register_nodes(factory, node);
    nodes::map::register_nodes(factory, node);
    nodes::approach::register_nodes(factory, node);
    nodes::actuators::register_nodes(factory, node);
    nodes::mission::register_nodes(factory, node);
}

}  // namespace robosub_mission
