#ifndef LEARNING_PERIOD__STATES_HPP_
#define LEARNING_PERIOD__STATES_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state_machine.hpp>

struct StateMachineConfig {
    std::string waypoint_manager_action_server;
    std::string waypoint_yaml_path;
};

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // LEARNING_PERIOD__STATES_HPP_
