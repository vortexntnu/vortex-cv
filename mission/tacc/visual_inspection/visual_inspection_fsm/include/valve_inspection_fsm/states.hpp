#ifndef VALVE_INSPECTION_FSM__STATES_HPP_
#define VALVE_INSPECTION_FSM__STATES_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state_machine.hpp>

struct StateMachineConfig {
    std::string landmark_convergence_action_server;
    std::string landmark_polling_action_server;
    std::string start_mission_service;
    std::string gripper_frame;
    std::string base_frame;
    double convergence_threshold;
    double dead_reckoning_threshold;
    double track_loss_timeout_sec;
};

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // VALVE_INSPECTION_FSM__STATES_HPP_
