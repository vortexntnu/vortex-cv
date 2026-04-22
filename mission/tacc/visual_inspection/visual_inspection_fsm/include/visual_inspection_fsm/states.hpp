#ifndef VISUAL_INSPECTION_FSM__STATES_HPP_
#define VISUAL_INSPECTION_FSM__STATES_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>

namespace valve_inspection_fsm {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

}  // namespace valve_inspection_fsm

struct StateMachineConfig {
    std::string waypoint_manager_action_server;
    std::string landmark_polling_action_server;
    std::string start_mission_service;
    std::string landmark_convergence_yaml_path;
    std::string standoff_waypoint_goal_id;
    std::string tcp_offset_goal_id;
    bool vertical_mounted_valve;
};

/**
 * @brief Moves drone to a standoff position aligned with the valve normal.
 *
 * Reads valve landmarks from the blackboard. Computes standoff position by
 * rotating the configured offset into the odom frame using the valve
 * orientation. Sets drone orientation so that +X faces −Z_valve (toward valve).
 */
class StandoffState : public yasmin_ros::ActionState<
                          valve_inspection_fsm::WaypointManagerAction> {
   public:
    StandoffState(const std::string& action_server_name,
                  vortex::utils::waypoints::WaypointGoal standoff_goal);

    valve_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::WaypointGoal standoff_goal_;
};

/**
 * @brief Aligns the gripper tip with the valve handle center.
 *
 * Reads refined valve landmarks from the blackboard. Rotates the configured
 * TCP offset (base_link → gripper_tip) into the odom frame using the drone
 * orientation, then computes base_link_target = valve_pos − rotated_offset.
 */
class ConvergeState : public yasmin_ros::ActionState<
                          valve_inspection_fsm::WaypointManagerAction> {
   public:
    ConvergeState(const std::string& action_server_name,
                  vortex::utils::waypoints::WaypointGoal standoff_goal,
                  vortex::utils::waypoints::WaypointGoal tcp_offset_goal);

    valve_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::WaypointGoal standoff_goal_;
    vortex::utils::waypoints::WaypointGoal tcp_offset_goal_;
};

/**
 * @brief Retreats along the valve outward normal after convergence.
 *
 * Reads refined valve landmarks from the blackboard. Computes retreat position
 * by rotating the standoff offset into odom frame, placing the drone safely
 * along +Z_valve away from the valve surface.
 */
class RetreatState : public yasmin_ros::ActionState<
                         valve_inspection_fsm::WaypointManagerAction> {
   public:
    RetreatState(const std::string& action_server_name,
                 vortex::utils::waypoints::WaypointGoal standoff_goal);

    valve_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::WaypointGoal standoff_goal_;
};

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // VISUAL_INSPECTION_FSM__STATES_HPP_
