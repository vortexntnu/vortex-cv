#include "valve_inspection_fsm/states.hpp"

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_yasmin_utils/landmark_converge_state.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

using vortex_yasmin_utils::LandmarkConvergeState;
using vortex_yasmin_utils::LandmarkPollingState;
using vortex_yasmin_utils::ServiceTriggerWaitState;
using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    vortex_msgs::msg::LandmarkType landmark_type;
    landmark_type.value = config.landmark_type_value;

    vortex_msgs::msg::LandmarkSubtype landmark_subtype;
    landmark_subtype.value = config.landmark_subtype_value;

    const auto convergence_goal =
        blackboard->get<vortex::utils::waypoints::LandmarkConvergenceGoal>(
            "landmark_convergence_goal");

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<ServiceTriggerWaitState>(config.start_mission_service),
        {{SUCCEED, "POLL_LANDMARK"}, {CANCEL, ABORT}});

    sm->add_state("POLL_LANDMARK",
                  std::make_shared<LandmarkPollingState>(
                      config.landmark_polling_action_server, landmark_type,
                      landmark_subtype, "landmarks", "landmark_found"),
                  {{"landmark_found", "LANDMARK_CONVERGENCE"}, {ABORT, ABORT}});

    sm->add_state("LANDMARK_CONVERGENCE",
                  std::make_shared<LandmarkConvergeState>(
                      config.landmark_convergence_action_server,
                      convergence_goal, landmark_type, landmark_subtype),
                  {{SUCCEED, "DONE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state(
        "DONE",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED},
            [](auto) {
                YASMIN_LOG_INFO("Visual inspection mission completed");
                return SUCCEED;
            }),
        {{SUCCEED, SUCCEED}});

    return sm;
}
