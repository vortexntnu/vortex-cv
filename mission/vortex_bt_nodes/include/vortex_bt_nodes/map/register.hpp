#ifndef VORTEX_BT_NODES__MAP__REGISTER_HPP_
#define VORTEX_BT_NODES__MAP__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes::map {

/**
 * @brief Register the map nodes. The landmark nodes (LandmarkKnown,
 * LandmarkConfirmed, GetLandmarkPose, GetApproachPose) share one
 * LandmarkCache of landmark_slam/landmarks.
 *
 * Planned: PoseFeeder, CourseFrameFeeder, SelectLandmark, Search,
 * AvoidSlalom, and the slalom nodes.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__REGISTER_HPP_
