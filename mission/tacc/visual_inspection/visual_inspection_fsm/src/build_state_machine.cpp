#include "visual_inspection_fsm/states.hpp"

#include <chrono>
#include <thread>

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto standoff_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "standoff_goal");
    const auto tcp_offset_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "tcp_offset_goal");

    vortex_msgs::msg::LandmarkType valve_type;
    valve_type.value = vortex_msgs::msg::LandmarkType::VALVE;

    vortex_msgs::msg::LandmarkSubtype valve_subtype;
    valve_subtype.value =
        config.vertical_mounted_valve
            ? vortex_msgs::msg::LandmarkSubtype::VALVE_VERTICAL
            : vortex_msgs::msg::LandmarkSubtype::VALVE_HORIZONTAL;

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_mission_service),
        {{SUCCEED, "LANDMARK_POLLING"}, {CANCEL, ABORT}});

    sm->add_state("LANDMARK_POLLING",
                  std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                      config.landmark_polling_action_server, valve_type,
                      valve_subtype, "valve_landmarks"),
                  {{"landmarks_found", "STANDOFF"}, {ABORT, ABORT}});

    sm->add_state("STANDOFF",
                  std::make_shared<StandoffState>(
                      config.waypoint_manager_action_server, standoff_goal),
                  {{SUCCEED, "STABILIZE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // Brief stabilization pause before re-polling for a refined estimate.
    sm->add_state("STABILIZE",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{SUCCEED},
                      [](auto) {
                          YASMIN_LOG_INFO("Stabilizing before refined poll...");
                          std::this_thread::sleep_for(std::chrono::seconds(5));
                          return SUCCEED;
                      }),
                  {{SUCCEED, "LANDMARK_POLLING_2"}});

    sm->add_state("LANDMARK_POLLING_2",
                  std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                      config.landmark_polling_action_server, valve_type,
                      valve_subtype, "valve_landmarks"),
                  {{"landmarks_found", "CONVERGE"}, {ABORT, ABORT}});

    sm->add_state(
        "CONVERGE",
        std::make_shared<ConvergeState>(config.waypoint_manager_action_server,
                                        standoff_goal, tcp_offset_goal),
        {{SUCCEED, "RETREAT"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state("RETREAT",
                  std::make_shared<RetreatState>(
                      config.waypoint_manager_action_server, standoff_goal),
                  {{SUCCEED, "DONE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state("DONE",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{SUCCEED},
                      [](auto) {
                          YASMIN_LOG_INFO("Valve inspection mission completed");
                          return SUCCEED;
                      }),
                  {{SUCCEED, SUCCEED}});

    return sm;
}
