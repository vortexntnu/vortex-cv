#include "visual_inspection_fsm/states.hpp"

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <vortex_yasmin_utils/gripper_state.hpp>
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
    valve_subtype.value = 0;

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
                  {{"landmarks_found", "ALIGN_HEIGHT_CAMERA"}, {ABORT, ABORT}});

    // Move to standoff XY and align camera height with the valve so the depth
    // camera looks directly at the valve before re-polling.
    sm->add_state(
        "ALIGN_HEIGHT_CAMERA",
        std::make_shared<AlignHeightCameraState>(
            config.waypoint_manager_action_server, standoff_goal,
            config.tcp_base_frame, config.depth_camera_frame),
        {{SUCCEED, "LANDMARK_POLLING_2"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // Re-poll with depth camera now level with the valve for the best reading.
    sm->add_state("LANDMARK_POLLING_2",
                  std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
                      config.landmark_polling_action_server, valve_type,
                      valve_subtype, "valve_landmarks"),
                  {{"landmarks_found", "ALIGN_HEIGHT"}, {ABORT, ABORT}});

    // Correct drone Z so the gripper tip will be at the valve handle height.
    // No further polling — the depth reading from LANDMARK_POLLING_2 is the
    // best estimate available.
    sm->add_state(
        "ALIGN_HEIGHT",
        std::make_shared<AlignHeightState>(
            config.waypoint_manager_action_server, standoff_goal,
            tcp_offset_goal, config.tcp_base_frame, config.tcp_tip_frame,
            config.valve_z_offset),
        {{SUCCEED, "OPEN_AND_ALIGN_GRIPPER"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state(
        "OPEN_AND_ALIGN_GRIPPER",
        std::make_shared<OpenAndAlignGripperState>(
            config.gripper_action_server, config.gripper_convergence_threshold),
        {{SUCCEED, "CONVERGE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state(
        "CONVERGE",
        std::make_shared<ConvergeState>(
            config.waypoint_manager_action_server, standoff_goal,
            tcp_offset_goal, config.tcp_base_frame, config.tcp_tip_frame,
            config.valve_z_offset),
        {{SUCCEED, "CLOSE_GRIPPER"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state("CLOSE_GRIPPER",
                  std::make_shared<vortex_yasmin_utils::GripperState>(
                      config.gripper_action_server,
                      0.0,  // roll — ignored in ONLY_PINCH mode
                      0.0,  // pinch = fully closed
                      vortex_msgs::msg::GripperWaypoint::ONLY_PINCH,
                      0.0005),  // tighter threshold so gripper is fully closed
                                // before twist
                  {{SUCCEED, "TWIST_HANDLE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    sm->add_state(
        "TWIST_HANDLE",
        std::make_shared<TwistHandleState>(config.gripper_action_server,
                                           config.gripper_convergence_threshold,
                                           config.valve_turn_direction),
        {{SUCCEED, "OPEN_GRIPPER"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // Release the handle before retreating so the gripper doesn't drag the
    // valve.
    sm->add_state("OPEN_GRIPPER",
                  std::make_shared<vortex_yasmin_utils::GripperState>(
                      config.gripper_action_server,
                      0.0,      // roll — ignored in ONLY_PINCH mode
                      -0.3333,  // pinch = fully open
                      vortex_msgs::msg::GripperWaypoint::ONLY_PINCH,
                      config.gripper_convergence_threshold),
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
