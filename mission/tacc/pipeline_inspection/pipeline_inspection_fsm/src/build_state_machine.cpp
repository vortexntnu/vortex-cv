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
    yasmin::Blackboard::SharedPtr blackboard,
    LatestPose latest_pose) {
    (void)blackboard;

    vortex_msgs::msg::LandmarkType pipeline_type;
    pipeline_type.value = vortex_msgs::msg::LandmarkType::PIPELINE_START;

    vortex_msgs::msg::LandmarkSubtype pipeline_subtype;
    pipeline_subtype.value =
        vortex_msgs::msg::LandmarkSubtype::PIPELINE_START_CAMERA;

    auto start_pipeline_trg = std::make_shared<
        yasmin_ros::ServiceState<pipeline_inspection_fsm::TriggerSrv>>(
        config.start_pipeline_following_service,
        [](yasmin::Blackboard::SharedPtr) {
            return std::make_shared<
                pipeline_inspection_fsm::TriggerSrv::Request>();
        });

    // Operator-triggered gate: localization hands off to this, and the
    // actual service call to the pipeline following node only fires once
    // we are told to proceed.
    auto wait_start_pipeline_following =
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_pipeline_following_trigger_service);

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
                {"WAIT_FOR_IRLS_LINE", {{SUCCEED, SUCCEED}, {CANCEL, ABORT}}}});
    };

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    // Snapshot the drone pose the moment the start-mission service is
    // triggered.
    auto store_origin = yasmin::CbState::make_shared(
        yasmin::Outcomes{SUCCEED, ABORT},
        [latest_pose](yasmin::Blackboard::SharedPtr bb) -> std::string {
            if (!latest_pose || !*latest_pose) {
                YASMIN_LOG_ERROR(
                    "STORE_ORIGIN: no odom received yet, cannot store origin "
                    "pose");
                return ABORT;
            }
            bb->set("origin_pose", **latest_pose);
            const auto& p = (*latest_pose)->position;
            YASMIN_LOG_INFO("STORE_ORIGIN: origin stored at [%.2f, %.2f, %.2f]",
                            p.x, p.y, p.z);
            return SUCCEED;
        });

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_mission_service),
        {{SUCCEED, "STORE_ORIGIN"}, {CANCEL, ABORT}});

    sm->add_state("STORE_ORIGIN", store_origin,
                  {{SUCCEED, "WIPE"}, {ABORT, ABORT}});

    if (config.start_above_pipe) {
        sm->add_state("WIPE",
                      std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "WAIT_START_PIPELINE_FOLLOWING"}});

    } else if (config.start_in_camera_range) {
        // Already in camera range: poll for pipeline endpoint landmarks,
        // collect pipeline bearing from the camera, then converge toward the
        // pipe.
        auto landmark_polling =
            std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                config.landmark_polling_action_server, pipeline_type,
                pipeline_subtype, "pipeline_landmarks", "landmark_found");

        auto pipeline_collect =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.pipeline_bearing_direction_action_server,
                config.pipeline_bearing_timeout_sec,
                config.pipeline_bearing_projection_distance,
                config.pipeline_bearing_min_measurements,
                config.pipeline_bearing_max_measurements,
                "pipeline_bearing_pose");

        auto pipeline_bearing_waypoint =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server, 0.5,
                config.bearing_waypoint_altitude, "pipeline_bearing_pose");

        sm->add_state("WIPE",
                      std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "LANDMARK_POLLING"}});
        sm->add_state(
            "LANDMARK_POLLING", landmark_polling,
            {{"landmark_found", "COLLECT_PIPELINE_BEARING"}, {ABORT, ABORT}});
        sm->add_state("COLLECT_PIPELINE_BEARING", pipeline_collect,
                      {{SUCCEED, "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("CONVERGE", make_converge_or_line(pipeline_bearing_waypoint),
                      {{SUCCEED, "WAIT_START_PIPELINE_FOLLOWING"},
                       {ABORT, ABORT},
                       {CANCEL, ABORT}});

    } else {
        // Normal mode:
        //   1. Collect acoustic bearing → drive 3 m at 2 m altitude.
        //   2. Collect acoustic bearing → drive 10 m at 2 m altitude, racing
        //      against landmark polling. Either the drive completes or the
        //      pipeline is spotted — both advance to pipeline bearing
        //      collection.
        //   3. Collect pipeline bearing from camera (ep1/ep2 endpoints).
        //   4. Converge toward the pipe at 1 m altitude, racing against IRLS
        //      line detection. Either way advances to pipeline following.

        auto acoustic_collect_short =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.bearing_direction_action_server,
                config.acoustic_short_timeout_sec,
                config.acoustic_bearing_projection_distance,
                config.acoustic_short_min_measurements,
                config.acoustic_short_max_measurements);

        auto drive_3m =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server, 0.5,
                config.acoustic_bearing_waypoint_altitude);

        auto acoustic_collect_long =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.bearing_direction_action_server,
                config.acoustic_long_timeout_sec,
                config.acoustic_search_projection_distance,
                config.acoustic_long_min_measurements,
                config.acoustic_long_max_measurements);

        auto drive_10m =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server, 0.5,
                config.acoustic_bearing_waypoint_altitude);

        auto landmark_polling =
            std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                config.landmark_polling_action_server, pipeline_type,
                pipeline_subtype, "pipeline_landmarks", "landmark_found");

        // Race: drive 10 m vs landmark polling — either outcome proceeds.
        auto hunt_10m =
            std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
                yasmin::StateMap{{"DRIVE_10M", drive_10m},
                                 {"LANDMARK_POLLING", landmark_polling}},
                ABORT,
                vortex_yasmin_utils::FirstWinsOutcomeMap{
                    {"DRIVE_10M",
                     {{SUCCEED, "pipeline_in_range"}, {ABORT, ABORT}}},
                    {"LANDMARK_POLLING",
                     {{"landmark_found", "pipeline_in_range"},
                      {ABORT, ABORT}}}});

        auto pipeline_collect =
            std::make_shared<vortex_yasmin_utils::CollectBearingDirectionState>(
                config.pipeline_bearing_direction_action_server,
                config.pipeline_bearing_timeout_sec,
                config.pipeline_bearing_projection_distance,
                config.pipeline_bearing_min_measurements,
                config.pipeline_bearing_max_measurements,
                "pipeline_bearing_pose");

        auto pipeline_bearing_waypoint =
            std::make_shared<vortex_yasmin_utils::BearingWaypointState>(
                config.waypoint_manager_action_server, 0.5,
                config.bearing_waypoint_altitude, "pipeline_bearing_pose");

        sm->add_state("WIPE",
                      std::make_shared<vortex_yasmin_utils::WipeState>(),
                      {{SUCCEED, "COLLECT_ACOUSTIC_1"}});
        sm->add_state("COLLECT_ACOUSTIC_1", acoustic_collect_short,
                      {{SUCCEED, "DRIVE_3M"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state(
            "DRIVE_3M", drive_3m,
            {{SUCCEED, "COLLECT_ACOUSTIC_2"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("COLLECT_ACOUSTIC_2", acoustic_collect_long,
                      {{SUCCEED, "HUNT_10M"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("HUNT_10M", hunt_10m,
                      {{"pipeline_in_range", "COLLECT_PIPELINE_BEARING"},
                       {ABORT, ABORT},
                       {CANCEL, ABORT}});
        sm->add_state("COLLECT_PIPELINE_BEARING", pipeline_collect,
                      {{SUCCEED, "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});
        sm->add_state("CONVERGE", make_converge_or_line(pipeline_bearing_waypoint),
                      {{SUCCEED, "WAIT_START_PIPELINE_FOLLOWING"},
                       {ABORT, ABORT},
                       {CANCEL, ABORT}});
    }

    // Hold here until an operator calls the pipeline-following trigger
    // service, then send the actual start service call to the pipeline
    // following node.
    sm->add_state(
        "WAIT_START_PIPELINE_FOLLOWING", wait_start_pipeline_following,
        {{SUCCEED, "WIPE_BEFORE_PIPELINE_FOLLOWING"}, {CANCEL, ABORT}});
    sm->add_state("WIPE_BEFORE_PIPELINE_FOLLOWING",
                  std::make_shared<vortex_yasmin_utils::WipeState>(),
                  {{SUCCEED, "START_PIPELINE_TRG"}});
    sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                  {{SUCCEED, "PIPELINE_FOLLOWING"}, {ABORT, ABORT}});

    std::shared_ptr<yasmin::State> pipeline_following;
    if (config.enable_end_detection) {
        pipeline_following = std::make_shared<
            vortex_yasmin_utils::FirstWinsConcurrence>(
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
    } else {
        pipeline_following =
            std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
                yasmin::StateMap{
                    {"PERSISTENT_WM",
                     std::make_shared<
                         vortex_yasmin_utils::PersistentWaypointManagerState>(
                         config.waypoint_manager_action_server)}},
                ABORT,
                vortex_yasmin_utils::FirstWinsOutcomeMap{
                    {"PERSISTENT_WM", {{SUCCEED, ABORT}, {ABORT, ABORT}}}},
                std::unordered_set<std::string>{});
    }

    vortex::utils::waypoints::WaypointGoal origin_wp;
    origin_wp.mode = vortex::utils::waypoints::WaypointMode::FORWARD_HEADING;
    origin_wp.convergence_threshold = 0.5;
    origin_wp.keep_altitude = true;
    origin_wp.desired_altitude = config.return_to_origin_altitude;

    auto return_to_origin = std::make_shared<SearchWaypointGoalState>(
        config.waypoint_manager_action_server,
        std::vector<vortex::utils::waypoints::WaypointGoal>{origin_wp});

    if (config.enable_end_detection) {
        sm->add_state(
            "PIPELINE_FOLLOWING", pipeline_following,
            {{SUCCEED, "RETURN_TO_ORIGIN"}, {CANCEL, ABORT}, {ABORT, ABORT}});
    } else {
        sm->add_state("PIPELINE_FOLLOWING", pipeline_following,
                      {{CANCEL, ABORT}, {ABORT, ABORT}});
    }

    sm->add_state("RETURN_TO_ORIGIN", return_to_origin,
                  {{SUCCEED, "DONE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

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
