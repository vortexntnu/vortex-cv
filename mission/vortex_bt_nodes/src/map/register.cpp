#include "vortex_bt_nodes/map/register.hpp"

#include <memory>

#include "vortex_bt_nodes/map/commit_target.hpp"
#include "vortex_bt_nodes/map/get_approach_pose.hpp"
#include "vortex_bt_nodes/map/get_landmark_pose.hpp"
#include "vortex_bt_nodes/map/go_to_frame.hpp"
#include "vortex_bt_nodes/map/go_to_pose.hpp"
#include "vortex_bt_nodes/map/landmark_cache.hpp"
#include "vortex_bt_nodes/map/landmark_confirmed.hpp"
#include "vortex_bt_nodes/map/landmark_known.hpp"
#include "vortex_bt_nodes/map/look_at_frame.hpp"
#include "vortex_bt_nodes/map/turn.hpp"

namespace vortex_bt_nodes::map {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    const std::shared_ptr<const LandmarkCache> cache =
        std::make_shared<LandmarkCache>(node);
    factory.registerNodeType<LandmarkKnown>("LandmarkKnown", cache);
    factory.registerNodeType<LandmarkConfirmed>("LandmarkConfirmed", cache);
    factory.registerNodeType<GetLandmarkPose>("GetLandmarkPose", cache);
    factory.registerNodeType<GetApproachPose>("GetApproachPose", cache);
    factory.registerNodeType<GoToFrame>("GoToFrame", node, cache);
    factory.registerNodeType<CommitTarget>("CommitTarget", cache);
    factory.registerNodeType<GoToPose>("GoToPose", node);
    factory.registerNodeType<Turn>("Turn", node, cache);
    factory.registerNodeType<LookAtFrame>("LookAtFrame", node, cache);
}

}  // namespace vortex_bt_nodes::map
