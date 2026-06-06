#include "pipeline_inspection_fsm/states.hpp"

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
#include <vortex_yasmin_utils/persistent_waypoint_manager_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>
#include <vortex_yasmin_utils/wipe_state.hpp>

using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    (void)blackboard;

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

    // Wraps a converge state in a race against the first IRLS line detection.
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

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_mission_service),
        {{SUCCEED, "WIPE"}, {CANCEL, ABORT}});

    if (config.start_above_pipe) {
        sm->add_state("WIPE", std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "START_PIPELINE_TRG"}});
        sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                      {{SUCCEED, "START_END_DETECTION_TRG"}, {ABORT, ABORT}});

    } else if (config.start_in_camera_range) {
        // Already in camera range: collect pipeline bearing, converge, follow.
        auto pipeline_collect =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.pipeline_bearing_direction_action_server,
                config.bearing_collection_timeout_sec,
                config.pipeline_bearing_projection_distance,
                config.bearing_min_measurements,
                "pipeline_bearing_pose");

        auto pipeline_bearing_waypoint =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server,
                0.5,
                config.bearing_waypoint_altitude,
                "pipeline_bearing_pose");

        sm->add_state("WIPE", std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "COLLECT_PIPELINE_BEARING"}});
        sm->add_state("COLLECT_PIPELINE_BEARING", pipeline_collect,
                      {{SUCCEED, "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("CONVERGE", make_converge_or_line(pipeline_bearing_waypoint),
                      {{SUCCEED, "START_PIPELINE_TRG"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                      {{SUCCEED, "START_END_DETECTION_TRG"}, {ABORT, ABORT}});

    } else {
        // Normal mode:
        //   Acoustic bearing loop (collect → 3m waypoint → repeat) runs
        //   concurrently with landmark polling. When the pipeline is spotted,
        //   landmark polling wins and cancels the acoustic loop.
        //   Then collect the pipeline bearing direction and converge.

        auto acoustic_collect =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.bearing_direction_action_server,
                config.bearing_collection_timeout_sec,
                config.acoustic_bearing_projection_distance,
                config.bearing_min_measurements);

        auto acoustic_waypoint =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server,
                0.5,
                config.bearing_waypoint_altitude);

        // Inner loop: collect acoustic bearing → send 3m waypoint → repeat
        auto acoustic_loop = std::make_shared<yasmin::StateMachine>(
            std::set<std::string>{ABORT});
        acoustic_loop->add_state(
            "COLLECT_ACOUSTIC_BEARING", acoustic_collect,
            {{SUCCEED, "SEND_ACOUSTIC_WAYPOINT"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        acoustic_loop->add_state(
            "SEND_ACOUSTIC_WAYPOINT", acoustic_waypoint,
            {{SUCCEED, "COLLECT_ACOUSTIC_BEARING"}, {ABORT, ABORT}, {CANCEL, ABORT}});

        auto landmark_polling =
            std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                config.landmark_polling_action_server, pipeline_type,
                pipeline_subtype, "pipeline_landmarks", "landmark_found");

        auto acoustic_hunt = std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
            yasmin::StateMap{
                {"ACOUSTIC_LOOP", acoustic_loop},
                {"LANDMARK_POLLING", landmark_polling}},
            ABORT,
            vortex_yasmin_utils::FirstWinsOutcomeMap{
                {"ACOUSTIC_LOOP", {{ABORT, ABORT}}},
                {"LANDMARK_POLLING",
                 {{"landmark_found", "landmark_found"}, {ABORT, ABORT}}}});

        auto pipeline_collect =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.pipeline_bearing_direction_action_server,
                config.bearing_collection_timeout_sec,
                config.pipeline_bearing_projection_distance,
                config.bearing_min_measurements,
                "pipeline_bearing_pose");

        auto pipeline_bearing_waypoint =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server,
                0.5,
                config.bearing_waypoint_altitude,
                "pipeline_bearing_pose");

        sm->add_state("WIPE", std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "ACOUSTIC_HUNT"}});
        sm->add_state("ACOUSTIC_HUNT", acoustic_hunt,
                      {{"landmark_found", "COLLECT_PIPELINE_BEARING"},
                       {ABORT, ABORT},
                       {CANCEL, ABORT}});
        sm->add_state("COLLECT_PIPELINE_BEARING", pipeline_collect,
                      {{SUCCEED, "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("CONVERGE", make_converge_or_line(pipeline_bearing_waypoint),
                      {{SUCCEED, "START_PIPELINE_TRG"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                      {{SUCCEED, "START_END_DETECTION_TRG"}, {ABORT, ABORT}});
    }

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
