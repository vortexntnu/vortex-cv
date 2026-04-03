#include "valve_inspection_fsm/states.hpp"
#include "valve_inspection_fsm/valve_converge_state.hpp"

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
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    vortex_msgs::msg::LandmarkType valve_type;
    valve_type.value = vortex_msgs::msg::LandmarkType::VALVE;

    vortex_msgs::msg::LandmarkSubtype valve_subtype;
    valve_subtype.value = vortex_msgs::msg::LandmarkSubtype::VALVE_VERTICAL;

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_mission_service),
        {{SUCCEED, "POLL_VALVE"}, {CANCEL, ABORT}});

    sm->add_state("POLL_VALVE",
                  std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                      config.landmark_polling_action_server, valve_type,
                      valve_subtype, "valve_landmarks", "valve_found"),
                  {{"valve_found", "CONVERGE_VALVE"}, {ABORT, ABORT}});

    sm->add_state(
        "CONVERGE_VALVE",
        std::make_shared<valve_inspection_fsm::ValveConvergeState>(
            config.landmark_convergence_action_server, valve_type,
            valve_subtype, config.convergence_threshold,
            config.dead_reckoning_threshold, config.track_loss_timeout_sec,
            config.gripper_frame, config.base_frame),
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
