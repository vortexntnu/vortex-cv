#ifndef VORTEX_BT_NODES__MAP__REGISTER_HPP_
#define VORTEX_BT_NODES__MAP__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace vortex_bt_nodes::map {

/**
 * @brief Register the map nodes (map and search, reading landmark_server).
 *
 * Planned: PoseFeeder, MapFeeder, CourseFrameFeeder, LandmarkKnown,
 * SelectLandmark, Search, MatchPipes, RecordLayer, AvoidSlalom.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__REGISTER_HPP_
