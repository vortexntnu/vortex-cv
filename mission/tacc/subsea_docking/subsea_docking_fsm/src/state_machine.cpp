#include "subsea_docking_fsm/states.hpp"

#include <yasmin/cb_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_msgs/srv/send_pose.hpp>

#include <vortex_yasmin_utils/first_wins_concurrence.hpp>
#include <vortex_yasmin_utils/landmark_converge_state.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_request_wait_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>
#include <vortex_yasmin_utils/waypoint_goal_state.hpp>

using vortex_yasmin_utils::FirstWinsConcurrence;
using vortex_yasmin_utils::FirstWinsOutcomeMap;
using vortex_yasmin_utils::LandmarkConvergeState;
using vortex_yasmin_utils::LandmarkPollingState;
using vortex_yasmin_utils::ServiceRequestWaitState;
using vortex_yasmin_utils::ServiceTriggerWaitState;
using vortex_yasmin_utils::WaypointGoalState;
using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    const auto fallback_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "fallback_waypoint_goal");

    const auto convergence_goal =
        blackboard->get<vortex::utils::waypoints::LandmarkConvergenceGoal>(
            "landmark_convergence_goal");

    vortex_msgs::msg::LandmarkType landmark_type;
    landmark_type.value = vortex_msgs::msg::LandmarkType::ARUCO_BOARD;

    vortex_msgs::msg::LandmarkSubtype landmark_subtype;
    landmark_subtype.value =
        0;  // Accept all subtypes under the specified type.

    sm->add_state(
        "START_MISSION_WAIT",
        std::make_shared<ServiceTriggerWaitState>(config.start_mission_service),
        {{SUCCEED, config.skip_search ? "LANDMARK_CONVERGENCE" : "SEARCH"},
         {CANCEL, ABORT}});

    if (!config.skip_search) {
        using SendPoseSrv = vortex_msgs::srv::SendPose;

        auto service_request =
            std::make_shared<ServiceRequestWaitState<SendPoseSrv>>(
                config.docking_position_service, "search_pose",
                std::chrono::duration<double>(
                    config.service_request_timeout_sec));

        auto landmark_polling = std::make_shared<LandmarkPollingState>(
            config.landmark_polling_action_server, landmark_type,
            landmark_subtype, "landmarks");

        auto search = std::make_shared<FirstWinsConcurrence>(
            yasmin::StateMap{{"SERVICE_REQUEST", service_request},
                             {"LANDMARK_POLLING", landmark_polling}},
            ABORT,
            FirstWinsOutcomeMap{
                {"SERVICE_REQUEST",
                 {{"timeout", "service_timeout"},
                  {SUCCEED, ABORT},
                  {CANCEL, ABORT}}},
                {"LANDMARK_POLLING",
                 {{"landmarks_found", "landmark_found"}, {ABORT, ABORT}}}});

        sm->add_state("SEARCH", search,
                      {{"landmark_found", "LANDMARK_CONVERGENCE"},
                       {"service_timeout", "FALLBACK_WAYPOINT"},
                       {ABORT, ABORT},
                       {CANCEL, ABORT}});

        sm->add_state("FALLBACK_WAYPOINT",
                      std::make_shared<WaypointGoalState>(
                          config.waypoint_manager_action_server, fallback_goal),
                      {{SUCCEED, "FALLBACK_LANDMARK_POLLING"}, {ABORT, ABORT}});

        sm->add_state(
            "FALLBACK_LANDMARK_POLLING",
            std::make_shared<LandmarkPollingState>(
                config.landmark_polling_action_server, landmark_type,
                landmark_subtype, "landmarks"),
            {{"landmarks_found", "LANDMARK_CONVERGENCE"}, {ABORT, ABORT}});
    }

    sm->add_state("LANDMARK_CONVERGENCE",
                  std::make_shared<LandmarkConvergeState>(
                      config.landmark_convergence_action_server,
                      convergence_goal, landmark_type, landmark_subtype),
                  {{SUCCEED, "DONE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state("DONE",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{SUCCEED},
                      [](auto) {
                          YASMIN_LOG_INFO("Subsea docking mission completed");
                          return SUCCEED;
                      }),
                  {{SUCCEED, SUCCEED}});

    return sm;
}
