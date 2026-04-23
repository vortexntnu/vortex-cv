#ifndef VISUAL_INSPECTION_FSM__STATES_HPP_
#define VISUAL_INSPECTION_FSM__STATES_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/gripper_reference_filter_waypoint.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>

namespace valve_inspection_fsm {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;
using GripperAction = vortex_msgs::action::GripperReferenceFilterWaypoint;

}  // namespace valve_inspection_fsm

struct StateMachineConfig {
    std::string waypoint_manager_action_server;
    std::string landmark_polling_action_server;
    std::string gripper_action_server;
    std::string start_mission_service;
    std::string landmark_convergence_yaml_path;
    std::string standoff_waypoint_goal_id;
    std::string tcp_offset_goal_id;
    bool vertical_mounted_valve;
    std::string tcp_base_frame;
    std::string tcp_tip_frame;
    double valve_z_offset{0.0};
    double gripper_convergence_threshold{0.05};
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
 * @brief Moves the drone to the correct height for convergence while staying at
 * standoff distance.
 *
 * Computes both the standoff position and the final converge target, then
 * commands the drone to (standoff_x, standoff_y, converge_z) so that height is
 * corrected before the depth approach begins.
 */
class AlignHeightState : public yasmin_ros::ActionState<
                             valve_inspection_fsm::WaypointManagerAction> {
   public:
    AlignHeightState(const std::string& action_server_name,
                     vortex::utils::waypoints::WaypointGoal standoff_goal,
                     vortex::utils::waypoints::WaypointGoal tcp_offset_goal,
                     std::string tcp_base_frame,
                     std::string tcp_tip_frame,
                     double valve_z_offset);

    valve_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::WaypointGoal standoff_goal_;
    vortex::utils::waypoints::WaypointGoal tcp_offset_goal_;
    std::string tcp_base_frame_;
    std::string tcp_tip_frame_;
    double valve_z_offset_;
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
                  vortex::utils::waypoints::WaypointGoal tcp_offset_goal,
                  std::string tcp_base_frame,
                  std::string tcp_tip_frame,
                  double valve_z_offset);

    valve_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex::utils::waypoints::WaypointGoal standoff_goal_;
    vortex::utils::waypoints::WaypointGoal tcp_offset_goal_;
    std::string tcp_base_frame_;
    std::string tcp_tip_frame_;
    double valve_z_offset_;
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

/**
 * @brief Opens the gripper and aligns its roll to the valve handle orientation.
 *
 * Reads valve landmarks from the blackboard. Extracts the valve yaw (Z rotation)
 * and clamps it to [0, π/2] — the physical handle range. Sends a ROLL_AND_PINCH
 * goal: pinch fully open (-0.3333) and roll matching the handle angle.
 * Stores the computed roll on the blackboard under "gripper_roll" so that
 * TwistHandleState can compute the opposing target later.
 *
 * Outcomes: SUCCEED, ABORT.
 */
class OpenAndAlignGripperState
    : public yasmin_ros::ActionState<valve_inspection_fsm::GripperAction> {
   public:
    OpenAndAlignGripperState(const std::string& action_server_name,
                             double convergence_threshold);

    valve_inspection_fsm::GripperAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string result_handler(
        yasmin::Blackboard::SharedPtr blackboard,
        valve_inspection_fsm::GripperAction::Result::SharedPtr result);

    void on_feedback(
        yasmin::Blackboard::SharedPtr blackboard,
        std::shared_ptr<const valve_inspection_fsm::GripperAction::Feedback>
            feedback);

   private:
    double convergence_threshold_;
    double computed_roll_{0.0};
    double actual_roll_{0.0};  // last roll reported by feedback — used for twist
};

/**
 * @brief Twists the valve handle 90° from its initial orientation.
 *
 * Reads "gripper_roll" from the blackboard (set by OpenAndAlignGripperState).
 * Computes target_roll = π/2 − stored_roll, which maps the two handle extremes
 * onto each other (0° ↔ 90°). Sends an ONLY_ROLL goal so pinch stays closed.
 *
 * Outcomes: SUCCEED, ABORT.
 */
class TwistHandleState
    : public yasmin_ros::ActionState<valve_inspection_fsm::GripperAction> {
   public:
    TwistHandleState(const std::string& action_server_name,
                     double convergence_threshold);

    valve_inspection_fsm::GripperAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string result_handler(
        yasmin::Blackboard::SharedPtr blackboard,
        valve_inspection_fsm::GripperAction::Result::SharedPtr result);

    void on_feedback(
        yasmin::Blackboard::SharedPtr blackboard,
        std::shared_ptr<const valve_inspection_fsm::GripperAction::Feedback>
            feedback);

   private:
    double convergence_threshold_;
};

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    rclcpp::Node::SharedPtr node,
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // VISUAL_INSPECTION_FSM__STATES_HPP_
