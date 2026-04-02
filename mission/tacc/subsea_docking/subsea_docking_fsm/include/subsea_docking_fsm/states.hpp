#ifndef SUBSEA_DOCKING_FSM__STATES_HPP_
#define SUBSEA_DOCKING_FSM__STATES_HPP_

#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_request_wait_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

#include <vortex_msgs/srv/send_pose.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state_machine.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

struct StateMachineConfig {
    bool skip_search;
    bool use_service_waypoint;
    std::string start_mission_service;
    std::string landmark_polling_action_server;
    std::string docking_position_service;
    std::string waypoint_yaml_path;
};

StateMachineConfig load_config(rclcpp::Node::SharedPtr node);

std::shared_ptr<yasmin::Blackboard> initialize_blackboard(
    const StateMachineConfig& config);

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard);

#endif  // SUBSEA_DOCKING_FSM__STATES_HPP_
