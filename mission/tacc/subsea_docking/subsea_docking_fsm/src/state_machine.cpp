#include "subsea_docking_fsm/states.hpp"
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>

#include <yasmin_ros/basic_outcomes.hpp>

using namespace vortex_yasmin_utils;
using namespace yasmin_ros::basic_outcomes;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    // -------------------------------------------------------------------------
    // START_MISSION_WAIT
    // Blocks until the mission start trigger service is called.
    // Routes directly to LANDMARK_POLLING if search is skipped, or if the
    // yaml-based waypoint path is used (no intermediate state needed there).
    // Routes to SERVICE_WAYPOINT_WAIT when a pose must be received via service.
    // -------------------------------------------------------------------------
    const std::string post_trigger_state =
        (!config.skip_search && config.use_service_waypoint)
            ? "SERVICE_WAYPOINT_WAIT"
            : "LANDMARK_POLLING";

    sm->add_state(
        "START_MISSION_WAIT",
        std::make_shared<ServiceTriggerWaitState>(config.start_mission_service),
        {{SUCCEED, post_trigger_state},
         {CANCEL, ABORT}});

    // -------------------------------------------------------------------------
    // SERVICE_WAYPOINT_WAIT  (only when skip_search=false, use_service_waypoint=true)
    // Advertises a SendPose service and blocks until a pose is received.
    // Stores the request on the blackboard under "search_pose".
    // -------------------------------------------------------------------------
    if (!config.skip_search && config.use_service_waypoint) {
        sm->add_state(
            "SERVICE_WAYPOINT_WAIT",
            std::make_shared<ServiceRequestWaitState<vortex_msgs::srv::SendPose>>(
                config.docking_position_service, "search_pose"),
            {{SUCCEED, "LANDMARK_POLLING"},
             {CANCEL, ABORT}});
    }

    vortex_msgs::msg::LandmarkType landmark_type;
    landmark_type.value = vortex_msgs::msg::LandmarkType::ARUCO_BOARD;

    vortex_msgs::msg::LandmarkSubtype landmark_subtype;
    landmark_subtype.value = 0; // Accept all subtypes under the specified type.

    sm->add_state(
        "LANDMARK_POLLING",
        std::make_shared<LandmarkPollingState>(
            config.landmark_polling_action_server,
            landmark_type,
            landmark_subtype,
            "landmarks"),
        {{"landmarks_found", SUCCEED},
         {ABORT, ABORT}});

    return sm;
}
