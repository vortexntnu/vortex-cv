#ifndef PIPELINE_INSPECTION_FSM__STATES_HPP_
#define PIPELINE_INSPECTION_FSM__STATES_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace pipeline_inspection_fsm {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;
using TriggerSrv = std_srvs::srv::Trigger;

}  // namespace pipeline_inspection_fsm

struct StateMachineConfig {
    std::string waypoint_manager_action_server;
    std::string landmark_polling_action_server;
    std::string start_mission_service;
    std::string start_pipeline_following_service;
    std::string end_of_pipeline_service;
    std::string waypoint_yaml_path;
    std::string convergence_yaml_path;
};

/**
 * @brief Sends all search waypoints in one WaypointManager action goal.
 *
 * Loads a vector of WaypointGoal (e.g. from load_waypoint_goal_from_yaml) and
 * sends them all at once as a single non-persistent WaypointManager goal.
 * Returns SUCCEED when all waypoints are visited, ABORT on failure.
 */
class SearchWaypointGoalState
    : public yasmin_ros::ActionState<
          pipeline_inspection_fsm::WaypointManagerAction> {
   public:
    SearchWaypointGoalState(
        const std::string& action_server_name,
        std::vector<vortex::utils::waypoints::WaypointGoal> waypoints);

    pipeline_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    std::vector<vortex::utils::waypoints::WaypointGoal> waypoints_;
};

/**
 * @brief Converges on the pipeline start by sending a single WaypointManager
 * goal computed from the detected landmark pose.
 *
 * Reads the first landmark from the blackboard key "pipeline_landmarks",
 * applies the configured pose offset via apply_pose_offset, then sends
 * the resulting target pose to the WaypointManager action server.
 */
class LandmarkConvergeState
    : public yasmin_ros::ActionState<
          pipeline_inspection_fsm::WaypointManagerAction> {
   public:
    LandmarkConvergeState(
        const std::string& action_server_name,
        vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal);

    pipeline_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::LandmarkConvergenceGoal convergence_goal_;
};

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // PIPELINE_INSPECTION_FSM__STATES_HPP_
