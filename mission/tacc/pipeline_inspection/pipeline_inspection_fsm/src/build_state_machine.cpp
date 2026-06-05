#include "pipeline_inspection_fsm/states.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <unordered_set>

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/service_state.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <vortex_yasmin_utils/bearing_waypoint_state.hpp>
#include <vortex_yasmin_utils/collect_bearing_direction_state.hpp>
#include <vortex_yasmin_utils/first_wins_concurrence.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/landmark_waypoint_state.hpp>
#include <vortex_yasmin_utils/persistent_waypoint_manager_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>
#include <vortex_yasmin_utils/wipe_state.hpp>

using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    vortex_msgs::msg::LandmarkType pipeline_type;
    pipeline_type.value = vortex_msgs::msg::LandmarkType::PIPELINE_START;

    vortex_msgs::msg::LandmarkSubtype pipeline_subtype;
    pipeline_subtype.value = vortex_msgs::msg::LandmarkSubtype::PIPELINE_START_CAMERA;

    auto start_pipeline_trg = std::make_shared<
        yasmin_ros::ServiceState<pipeline_inspection_fsm::TriggerSrv>>(
        config.start_pipeline_following_service,
        [](yasmin::Blackboard::SharedPtr) {
            return std::make_shared<pipeline_inspection_fsm::TriggerSrv::Request>();
        });

    auto start_end_detection_trg = std::make_shared<
        yasmin_ros::ServiceState<pipeline_inspection_fsm::TriggerSrv>>(
        config.start_end_pipeline_detection_service,
        [](yasmin::Blackboard::SharedPtr) {
            return std::make_shared<pipeline_inspection_fsm::TriggerSrv::Request>();
        });

    // Wraps a convergence waypoint state so that it runs concurrently with a
    // wait for the first IRLS line. Whichever finishes first wins: the vehicle
    // starts converging on the pipe, but we do NOT require the (likely
    // overshooting) convergence waypoint to be reached -- as soon as the first
    // length-gated IRLS line is published, an external trigger node calls
    // irls_line_detected_service and we proceed to pipeline following.
    auto make_converge_or_line =
        [&config](std::shared_ptr<yasmin::State> converge_state)
        -> std::shared_ptr<vortex_yasmin_utils::FirstWinsConcurrence> {
        auto wait_line =
            std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
                config.irls_line_detected_service);
        return std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
            yasmin::StateMap{{"CONVERGE", converge_state},
                             {"WAIT_FOR_IRLS_LINE", wait_line}},
            ABORT,
            vortex_yasmin_utils::FirstWinsOutcomeMap{
                {"CONVERGE",
                 {{SUCCEED, SUCCEED}, {ABORT, ABORT}, {CANCEL, ABORT}}},
                {"WAIT_FOR_IRLS_LINE",
                 {{SUCCEED, SUCCEED}, {CANCEL, ABORT}}}});
    };

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    // Always wait for the start trigger and wipe before entering any active state.
    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_mission_service),
        {{SUCCEED, "WIPE"}, {CANCEL, ABORT}});

    if (config.start_above_pipe) {
        // Already above the pipe at the right altitude: trigger pipeline following directly.
        sm->add_state("WIPE", std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "START_PIPELINE_TRG"}});
        sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                      {{SUCCEED, "START_END_DETECTION_TRG"}, {ABORT, ABORT}});

    } else if (config.start_in_camera_range) {
        // Already in camera range: poll for landmark, converge, then follow.
        const auto convergence_goal =
            blackboard->get<vortex::utils::waypoints::WaypointGoal>("convergence_goal");

        auto landmark_polling =
            std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                config.landmark_polling_action_server, pipeline_type,
                pipeline_subtype, "pipeline_landmarks", "landmark_found");
        auto converge =
            std::make_shared<vortex_yasmin_utils::LandmarkWaypointState>(
                config.waypoint_manager_action_server, convergence_goal,
                "pipeline_landmarks");

        sm->add_state("WIPE", std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "LANDMARK_POLLING"}});
        sm->add_state("LANDMARK_POLLING", landmark_polling,
                      {{"landmark_found", "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("CONVERGE", make_converge_or_line(converge),
                      {{SUCCEED, "START_PIPELINE_TRG"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                      {{SUCCEED, "START_END_DETECTION_TRG"}, {ABORT, ABORT}});

    } else {
        // Normal mode: collect bearing, navigate to bearing waypoint, search, converge, follow.
        auto collect_bearing =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.bearing_direction_action_server,
                config.bearing_collection_timeout_sec,
                config.bearing_projection_distance);

        auto send_bearing_waypoint =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server);

        const auto search_waypoints =
            blackboard->get<std::vector<vortex::utils::waypoints::WaypointGoal>>(
                "search_waypoints");
        const auto convergence_goal =
            blackboard->get<vortex::utils::waypoints::WaypointGoal>("convergence_goal");

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
        auto converge =
            std::make_shared<vortex_yasmin_utils::LandmarkWaypointState>(
                config.waypoint_manager_action_server, convergence_goal,
                "pipeline_landmarks");

        sm->add_state("WIPE", std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "COLLECT_BEARING"}});
        sm->add_state("COLLECT_BEARING", collect_bearing,
                      {{SUCCEED, "SEND_BEARING_WAYPOINT"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("SEND_BEARING_WAYPOINT", send_bearing_waypoint,
                      {{SUCCEED, "SEARCH"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("SEARCH", search,
                      {{"landmark_found", "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("CONVERGE", make_converge_or_line(converge),
                      {{SUCCEED, "START_PIPELINE_TRG"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                      {{SUCCEED, "START_END_DETECTION_TRG"}, {ABORT, ABORT}});
    }

    // The 30s settling delay now lives inside the pipeline_end_detector node
    // (activation_delay_sec), so the FSM triggers detection immediately and
    // proceeds straight into pipeline following — opening the persistent
    // waypoint manager goal without first blocking for the timeout.
    sm->add_state("START_END_DETECTION_TRG", start_end_detection_trg,
                  {{SUCCEED, "PIPELINE_FOLLOWING"}, {ABORT, ABORT}});

    auto pipeline_following =
        std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
            yasmin::StateMap{
                {"PERSISTENT_WM",
                 std::make_shared<vortex_yasmin_utils::PersistentWaypointManagerState>(
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
