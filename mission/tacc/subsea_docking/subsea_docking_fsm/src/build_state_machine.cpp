#include "subsea_docking_fsm/states.hpp"

#include <chrono>
#include <thread>
#include <unordered_set>

#include <yasmin/cb_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <vortex_msgs/srv/send_pose.hpp>

#include <spdlog/spdlog.h>
#include <std_srvs/srv/trigger.hpp>
#include <vortex_yasmin_utils/first_wins_concurrence.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/landmark_waypoint_state.hpp>
#include <vortex_yasmin_utils/service_request_wait_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>
#include <vortex_yasmin_utils/waypoint_goal_state.hpp>
#include <yasmin_ros/yasmin_node.hpp>

using vortex_yasmin_utils::FirstWinsConcurrence;
using vortex_yasmin_utils::FirstWinsOutcomeMap;
using vortex_yasmin_utils::LandmarkPollingState;
using vortex_yasmin_utils::LandmarkWaypointState;
using vortex_yasmin_utils::ServiceRequestWaitState;
using vortex_yasmin_utils::ServiceTriggerWaitState;
using vortex_yasmin_utils::WaypointGoalState;
using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

class StartMissionWaitState
    : public vortex_yasmin_utils::ServiceTriggerWaitState {
   public:
    StartMissionWaitState(const std::string& service_name,
                          const std::string& docking_estimator_service)
        : ServiceTriggerWaitState(service_name) {
        auto node = yasmin_ros::YasminNode::get_instance();
        docking_start_client_ = node->create_client<std_srvs::srv::Trigger>(
            docking_estimator_service);
    }

   protected:
    void on_triggered(yasmin::Blackboard::SharedPtr) override {
        if (!docking_start_client_->service_is_ready()) {
            spdlog::warn(
                "[StartMission] Docking estimator start service not available");
            return;
        }
        docking_start_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        spdlog::info("[StartMission] Sent start signal to docking estimator");
    }

   private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_start_client_;
};

class SearchPoseWaypointState : public vortex_yasmin_utils::WaypointGoalState {
   public:
    using WaypointManagerAction = vortex_msgs::action::WaypointManager;

    SearchPoseWaypointState(const std::string& action_server_name,
                            double convergence_threshold)
        : WaypointGoalState(action_server_name, {}),
          convergence_threshold_(convergence_threshold) {}

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard) override {
        auto request =
            blackboard->get<vortex_msgs::srv::SendPose::Request::SharedPtr>(
                "search_pose");

        vortex_msgs::msg::Waypoint wp;
        wp.pose = request->pose.pose;
        wp.waypoint_mode.mode = vortex_msgs::msg::WaypointMode::ONLY_POSITION;

        WaypointManagerAction::Goal goal;
        goal.waypoints = {wp};
        goal.persistent = false;
        goal.convergence_threshold = convergence_threshold_;
        return goal;
    }

   private:
    double convergence_threshold_;
};

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    const auto dock_config_waypoint_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "dock_config_waypoint_goal");

    const auto above_dock_waypoint_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "above_dock_waypoint_goal");

    const auto power_puck_waypoint_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "power_puck_waypoint_goal");

    vortex_msgs::msg::LandmarkType dock_landmark_type;
    dock_landmark_type.value = vortex_msgs::msg::LandmarkType::ARUCO_BOARD;

    vortex_msgs::msg::LandmarkSubtype dock_landmark_subtype;
    dock_landmark_subtype.value =
        0;  // Accept all subtypes under the specified type.

    sm->add_state(
        "AWAIT_START_TRIGGER",
        std::make_shared<StartMissionWaitState>(
            config.start_mission_service,
            config.docking_estimator_start_service),
        {{SUCCEED, config.use_wall_detection ? "WALL_DETECTION_ESTIMATE"
                                             : "DOCK_CONFIG_WAYPOINT"},
         {CANCEL, ABORT}});

    auto dock_config_nav = std::make_shared<WaypointGoalState>(
        config.waypoint_manager_action_server, dock_config_waypoint_goal);

    auto dock_config_landmark_poll = std::make_shared<LandmarkPollingState>(
        config.landmark_polling_action_server, dock_landmark_type,
        dock_landmark_subtype, "landmarks");

    auto dock_config_concurrent = std::make_shared<FirstWinsConcurrence>(
        yasmin::StateMap{
            {"DOCK_CONFIG_NAV", dock_config_nav},
            {"DOCK_CONFIG_LANDMARK_POLL", dock_config_landmark_poll}},
        ABORT,
        FirstWinsOutcomeMap{
            {"DOCK_CONFIG_NAV", {{SUCCEED, ABORT}, {ABORT, ABORT}}},
            {"DOCK_CONFIG_LANDMARK_POLL",
             {{"landmarks_found", "landmark_found"}, {ABORT, ABORT}}}},
        std::unordered_set<std::string>{"DOCK_CONFIG_LANDMARK_POLL"});

    sm->add_state("DOCK_CONFIG_WAYPOINT", dock_config_concurrent,
                  {{"landmark_found", "ABOVE_DOCK_WAYPOINT"},
                   {ABORT, ABORT},
                   {CANCEL, ABORT}});

    if (config.use_wall_detection) {
        using SendPoseSrv = vortex_msgs::srv::SendPose;

        auto service_request =
            std::make_shared<ServiceRequestWaitState<SendPoseSrv>>(
                config.docking_position_service, "search_pose",
                std::chrono::duration<double>(
                    config.service_request_timeout_sec));

        auto landmark_polling = std::make_shared<LandmarkPollingState>(
            config.landmark_polling_action_server, dock_landmark_type,
            dock_landmark_subtype, "landmarks");

        auto search = std::make_shared<FirstWinsConcurrence>(
            yasmin::StateMap{{"SERVICE_REQUEST", service_request},
                             {"LANDMARK_POLLING", landmark_polling}},
            ABORT,
            FirstWinsOutcomeMap{{"SERVICE_REQUEST",
                                 {{"timeout", "service_timeout"},
                                  {SUCCEED, "pose_received"},
                                  {CANCEL, ABORT}}},
                                {"LANDMARK_POLLING",
                                 {{"landmarks_found", "landmark_found"},
                                  {ABORT, "service_timeout"}}}});

        sm->add_state("WALL_DETECTION_ESTIMATE", search,
                      {{"landmark_found", "ABOVE_DOCK_WAYPOINT"},
                       {"pose_received", "NAV_TO_WALL_DETECT_WAYPOINT"},
                       {"service_timeout", "DOCK_CONFIG_WAYPOINT"},
                       {ABORT, ABORT},
                       {CANCEL, ABORT}});

        auto nav_to_search_pose = std::make_shared<SearchPoseWaypointState>(
            config.waypoint_manager_action_server, 0.5);

        auto nav_landmark_poll = std::make_shared<LandmarkPollingState>(
            config.landmark_polling_action_server, dock_landmark_type,
            dock_landmark_subtype, "landmarks");

        auto nav_concurrent = std::make_shared<FirstWinsConcurrence>(
            yasmin::StateMap{{"SEARCH_POSE_NAV", nav_to_search_pose},
                             {"NAV_LANDMARK_POLL", nav_landmark_poll}},
            ABORT,
            FirstWinsOutcomeMap{
                {"SEARCH_POSE_NAV", {{SUCCEED, "nav_done"}, {ABORT, ABORT}}},
                {"NAV_LANDMARK_POLL",
                 {{"landmarks_found", "landmark_found"}, {ABORT, ABORT}}}});

        sm->add_state("NAV_TO_WALL_DETECT_WAYPOINT", nav_concurrent,
                      {{"landmark_found", "ABOVE_DOCK_WAYPOINT"},
                       {"nav_done", "DOCK_CONFIG_WAYPOINT"},
                       {ABORT, "DOCK_CONFIG_WAYPOINT"},
                       {CANCEL, ABORT}});
    }

    // Navigate above the docking station using the polled landmark position.
    sm->add_state(
        "ABOVE_DOCK_WAYPOINT",
        std::make_shared<LandmarkWaypointState>(
            config.waypoint_manager_action_server, above_dock_waypoint_goal,
            "landmarks"),
        {{SUCCEED, "STABILIZE_ABOVE_DOCK"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // Hold position for 5 seconds before fine-tuning horizontal alignment.
    sm->add_state("STABILIZE_ABOVE_DOCK",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{SUCCEED},
                      [](auto) {
                          YASMIN_LOG_INFO("Stabilizing for 5 seconds...");
                          std::this_thread::sleep_for(std::chrono::seconds(5));
                          return SUCCEED;
                      }),
                  {{SUCCEED, "ALIGN_ABOVE_DOCK"}});

    // Fine-tune x/y and yaw above the dock while holding depth.
    vortex::utils::waypoints::WaypointGoal above_dock_xy_goal =
        above_dock_waypoint_goal;
    above_dock_xy_goal.mode = vortex::utils::types::WaypointMode::XY_AND_YAW;

    sm->add_state(
        "ALIGN_ABOVE_DOCK",
        std::make_shared<LandmarkWaypointState>(
            config.waypoint_manager_action_server, above_dock_xy_goal,
            "landmarks"),
        {{SUCCEED, "STABILIZE_ALIGNED"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // Hold position for 5 seconds after alignment before re-polling.
    sm->add_state(
        "STABILIZE_ALIGNED",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED},
            [](auto) {
                YASMIN_LOG_INFO("Stabilizing after alignment for 5 seconds...");
                std::this_thread::sleep_for(std::chrono::seconds(5));
                return SUCCEED;
            }),
        {{SUCCEED, "LANDMARK_POLLING_ABOVE_DOCK"}});

    // Re-poll to get a fresh landmark estimate after stabilization.
    sm->add_state("LANDMARK_POLLING_ABOVE_DOCK",
                  std::make_shared<LandmarkPollingState>(
                      config.landmark_polling_action_server, dock_landmark_type,
                      dock_landmark_subtype, "landmarks"),
                  {{"landmarks_found", "DOCKING_CONVERGENCE"}, {ABORT, ABORT}});

    // Converge on the docking station using the fresh landmark + power puck
    // offset.
    sm->add_state("DOCKING_CONVERGENCE",
                  std::make_shared<LandmarkWaypointState>(
                      config.waypoint_manager_action_server,
                      power_puck_waypoint_goal, "landmarks"),
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
