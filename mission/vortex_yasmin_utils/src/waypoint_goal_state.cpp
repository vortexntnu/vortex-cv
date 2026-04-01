#include "vortex_yasmin_utils/waypoint_goal_state.hpp"

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <yasmin_ros/basic_outcomes.hpp>

namespace vortex_yasmin_utils {

WaypointGoalState::WaypointGoalState(
    const std::string& action_server_name,
    vortex::utils::waypoints::WaypointGoal waypoint_goal)
    : ActionState(
          action_server_name,
          std::bind(&WaypointGoalState::create_goal, this,
                    std::placeholders::_1),
          yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED,
                           yasmin_ros::basic_outcomes::ABORT},
          std::bind(&WaypointGoalState::result_handler, this,
                    std::placeholders::_1, std::placeholders::_2)),
      waypoint_goal_(std::move(waypoint_goal)) {}

WaypointManagerAction::Goal WaypointGoalState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    vortex_msgs::msg::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::to_pose_msg(waypoint_goal_.pose);
    wp.waypoint_mode.mode = static_cast<uint8_t>(waypoint_goal_.mode);

    WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = waypoint_goal_.convergence_threshold;

    return goal;
}

std::string WaypointGoalState::result_handler(
    yasmin::Blackboard::SharedPtr /*blackboard*/,
    WaypointManagerAction::Result::SharedPtr result) {
    return result->success ? yasmin_ros::basic_outcomes::SUCCEED
                           : yasmin_ros::basic_outcomes::ABORT;
}

}  // namespace vortex_yasmin_utils
