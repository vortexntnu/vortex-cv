#include "pipeline_inspection_fsm/states.hpp"

#include <unordered_set>

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/service_state.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <vortex_yasmin_utils/first_wins_concurrence.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/persistent_waypoint_manager_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto search_waypoints =
        blackboard->get<std::vector<vortex::utils::waypoints::WaypointGoal>>(
            "search_waypoints");

    const auto convergence_goal =
        blackboard->get<vortex::utils::waypoints::LandmarkConvergenceGoal>(
            "convergence_goal");

    vortex_msgs::msg::LandmarkType pipeline_type;
    pipeline_type.value = 1;

    vortex_msgs::msg::LandmarkSubtype pipeline_subtype;
    pipeline_subtype.value = 19;

    auto search_pattern = std::make_shared<SearchWaypointGoalState>(
        config.waypoint_manager_action_server, search_waypoints);

    auto landmark_polling =
        std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
            config.landmark_polling_action_server, pipeline_type,
            pipeline_subtype, "pipeline_landmarks", "landmark_found");

    auto search = std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
        yasmin::StateMap{{"SEARCH_PATTERN", search_pattern},
                         {"LANDMARK_POLLING", landmark_polling}},
        ABORT,
        vortex_yasmin_utils::FirstWinsOutcomeMap{
            {"SEARCH_PATTERN", {{SUCCEED, ABORT}}},
            {"LANDMARK_POLLING", {{"landmark_found", "landmark_found"}}}});

    auto converge = std::make_shared<LandmarkConvergeState>(
        config.waypoint_manager_action_server, convergence_goal);

    auto start_pipeline_trg = std::make_shared<
        yasmin_ros::ServiceState<pipeline_inspection_fsm::TriggerSrv>>(
        config.start_pipeline_following_service,
        [](yasmin::Blackboard::SharedPtr) {
            return std::make_shared<
                pipeline_inspection_fsm::TriggerSrv::Request>();
        });

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_mission_service),
        {{SUCCEED, "SEARCH"}, {CANCEL, ABORT}});

    sm->add_state(
        "SEARCH", search,
        {{"landmark_found", "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state(
        "CONVERGE", converge,
        {{SUCCEED, "START_PIPELINE_TRG"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                  {{SUCCEED, "PIPELINE_FOLLOWING"}, {ABORT, ABORT}});

    auto pipeline_following =
        std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
            yasmin::StateMap{
                {"PERSISTENT_WM",
                 std::make_shared<
                     vortex_yasmin_utils::PersistentWaypointManagerState>(
                     config.waypoint_manager_action_server)},
                {"WAIT_FOR_END_OF_PIPELINE",
                 std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
                     config.end_of_pipeline_service)}},
            ABORT,
            vortex_yasmin_utils::FirstWinsOutcomeMap{
                {"PERSISTENT_WM", {{SUCCEED, ABORT}, {ABORT, ABORT}}},
                {"WAIT_FOR_END_OF_PIPELINE",
                 {{SUCCEED, SUCCEED}, {CANCEL, CANCEL}}}},
            std::unordered_set<std::string>{"WAIT_FOR_END_OF_PIPELINE"});

    sm->add_state("PIPELINE_FOLLOWING", pipeline_following,
                  {{SUCCEED, "DONE"}, {CANCEL, ABORT}, {ABORT, ABORT}});

    sm->add_state(
        "DONE",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED},
            [](auto) {
                YASMIN_LOG_INFO("Pipeline inspection mission completed");
                return SUCCEED;
            }),
        {{SUCCEED, SUCCEED}});

    return sm;
}
