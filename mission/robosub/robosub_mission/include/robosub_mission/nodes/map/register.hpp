#ifndef ROBOSUB_MISSION__NODES__MAP__REGISTER_HPP_
#define ROBOSUB_MISSION__NODES__MAP__REGISTER_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace robosub_mission::nodes::map {

/**
 * @brief Register the map nodes (map and search, reading landmark_server).
 *
 * Planned: PoseFeeder, MapFeeder, CourseFrameFeeder, LandmarkKnown,
 * SelectLandmark, Search, MatchPipes, RecordLayer, AvoidSlalom.
 */
void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node);

}  // namespace robosub_mission::nodes::map

#endif  // ROBOSUB_MISSION__NODES__MAP__REGISTER_HPP_
