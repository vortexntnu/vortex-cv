#include "vortex_bt_nodes/map/register.hpp"

#include <memory>

<<<<<<< HEAD
#include "vortex_bt_nodes/map/landmark_known.hpp"
=======
#include "vortex_bt_nodes/map/go_to_frame_node.hpp"
#include "vortex_bt_nodes/map/landmark_map.hpp"
>>>>>>> b3805cb (GoToFrame created - compiles, but not tested)

namespace vortex_bt_nodes::map {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    // One map for all map nodes. Pass it to the nodes that need it.
<<<<<<< HEAD
    const std::shared_ptr<const LandmarkMap> map =
        std::make_shared<LandmarkMap>(node);

    factory.registerNodeType<LandmarkKnown>("LandmarkKnown", map);
=======
    const std::shared_ptr<LandmarkMap> map =
        std::make_shared<LandmarkMap>(node);
    factory.registerNodeType<GoToFrame>("GoToFrame", node, map);
>>>>>>> b3805cb (GoToFrame created - compiles, but not tested)
}

}  // namespace vortex_bt_nodes::map
