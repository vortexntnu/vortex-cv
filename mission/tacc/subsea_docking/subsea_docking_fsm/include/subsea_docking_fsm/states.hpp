#ifndef SUBSEA_DOCKING_FSM__STATES_HPP_
#define SUBSEA_DOCKING_FSM__STATES_HPP_

#include <vortex_msgs/srv/send_pose.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state_machine.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

struct StateMachineConfig {
    bool skip_search;
    bool use_service_waypoint;
    std::string start_mission_service;
    std::string waypoint_manager_action_server;
    std::string landmark_convergence_action_server;
    std::string landmark_polling_action_server;
    std::string waypoint_yaml_path;
    std::string landmark_convergence_yaml_path;
    double service_request_timeout_sec;
    std::string fallback_waypoint_id;
    std::string landmark_convergence_goal_id;
    std::string docking_position_service;
};

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // SUBSEA_DOCKING_FSM__STATES_HPP_
