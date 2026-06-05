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
    std::string bearing_direction_action_server;
    std::string start_mission_service;
    std::string start_pipeline_following_service;
    std::string start_end_pipeline_detection_service;
    std::string end_of_pipeline_service;
    std::string waypoint_yaml_path;
    std::string convergence_yaml_path;
    double bearing_collection_timeout_sec{10.0};
    double bearing_projection_distance{10.0};
    bool start_in_camera_range{false};
    bool start_above_pipe{false};
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

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // PIPELINE_INSPECTION_FSM__STATES_HPP_
