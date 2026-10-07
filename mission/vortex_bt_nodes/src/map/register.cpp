#include "vortex_bt_nodes/map/register.hpp"

#include <memory>

#include "vortex_bt_nodes/map/get_approach_pose.hpp"
#include "vortex_bt_nodes/map/get_landmark_pose.hpp"
#include "vortex_bt_nodes/map/landmark_cache.hpp"
#include "vortex_bt_nodes/map/landmark_confirmed.hpp"
#include "vortex_bt_nodes/map/landmark_known.hpp"

namespace vortex_bt_nodes::map {

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    // One map for every landmark node of the tree.
    const std::shared_ptr<const LandmarkCache> cache =
        std::make_shared<LandmarkCache>(node);
    factory.registerNodeType<LandmarkKnown>("LandmarkKnown", cache);
    factory.registerNodeType<LandmarkConfirmed>("LandmarkConfirmed", cache);
    factory.registerNodeType<GetLandmarkPose>("GetLandmarkPose", cache);
    factory.registerNodeType<GetApproachPose>("GetApproachPose", cache);
}

}  // namespace vortex_bt_nodes::map
