#include "pipeline_inspection_fsm/states.hpp"

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/service_state.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <vortex_yasmin_utils/first_wins_concurrence.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static std::vector<vortex::utils::waypoints::WaypointGoal>
load_search_waypoints(const std::string& path) {
    std::vector<vortex::utils::waypoints::WaypointGoal> waypoints;
    for (int i = 1;; ++i) {
        try {
            waypoints.push_back(
                vortex::utils::waypoints::load_waypoint_goal_from_yaml(
                    path, "search_waypoint_" + std::to_string(i)));
        } catch (const std::runtime_error&) {
            break;
        }
    }
    return waypoints;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    yasmin_ros::set_ros_loggers();

    YASMIN_LOG_INFO("Starting Pipeline Inspection FSM");

    auto blackboard = initialize_blackboard();

    // Load search waypoints and convergence goal at startup
    const auto search_waypoints = load_search_waypoints(
        blackboard->get<std::string>("waypoint_file_path"));

    if (search_waypoints.empty()) {
        YASMIN_LOG_ERROR("No search waypoints configured, aborting");
        rclcpp::shutdown();
        return 1;
    }

    const auto convergence_goal =
        vortex::utils::waypoints::load_landmark_goal_from_yaml(
            blackboard->get<std::string>("convergence_file_path"),
            "pipeline_start_convergence");

    vortex_msgs::msg::LandmarkType pipeline_type;
    pipeline_type.value = 1;

    vortex_msgs::msg::LandmarkSubtype pipeline_subtype;
    pipeline_subtype.value = 19;

    auto search_pattern = std::make_shared<SearchWaypointGoalState>(
        blackboard->get<std::string>("action_server.waypoint_manager"),
        search_waypoints);

    auto landmark_polling =
        std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
            blackboard->get<std::string>("action_server.landmark_polling"),
            pipeline_type, pipeline_subtype, "pipeline_landmarks",
            "landmark_found");

    auto search = std::make_shared<vortex_yasmin_utils::FirstWinsConcurrence>(
        yasmin::StateMap{{"SEARCH_PATTERN", search_pattern},
                         {"LANDMARK_POLLING", landmark_polling}},
        yasmin_ros::basic_outcomes::ABORT,
        vortex_yasmin_utils::FirstWinsOutcomeMap{
            {"SEARCH_PATTERN",
             {{yasmin_ros::basic_outcomes::SUCCEED,
               yasmin_ros::basic_outcomes::ABORT}}},
            {"LANDMARK_POLLING", {{"landmark_found", "landmark_found"}}}});

    auto converge = std::make_shared<LandmarkConvergeState>(
        blackboard->get<std::string>("action_server.waypoint_manager"),
        convergence_goal);

    auto start_pipeline_trg = std::make_shared<
        yasmin_ros::ServiceState<pipeline_inspection_fsm::TriggerSrv>>(
        blackboard->get<std::string>("service.start_pipeline_following"),
        [](yasmin::Blackboard::SharedPtr) {
            return std::make_shared<
                pipeline_inspection_fsm::TriggerSrv::Request>();
        });

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                              yasmin_ros::basic_outcomes::ABORT});

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            blackboard->get<std::string>("service.start_mission")),
        {{yasmin_ros::basic_outcomes::SUCCEED, "SEARCH"},
         {yasmin_ros::basic_outcomes::CANCEL,
          yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state(
        "SEARCH", search,
        {{"landmark_found", "CONVERGE"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
         {yasmin_ros::basic_outcomes::CANCEL,
          yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state(
        "CONVERGE", converge,
        {{yasmin_ros::basic_outcomes::SUCCEED, "START_PIPELINE_TRG"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
         {yasmin_ros::basic_outcomes::CANCEL,
          yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                  {{yasmin_ros::basic_outcomes::SUCCEED, "START_WM"},
                   {yasmin_ros::basic_outcomes::ABORT,
                    yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state("START_WM",
                  std::make_shared<StartWaypointManagerState>(blackboard),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "PIPELINE_FOLLOWING"},
                   {yasmin_ros::basic_outcomes::ABORT,
                    yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state(
        "PIPELINE_FOLLOWING",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            blackboard->get<std::string>("service.end_of_pipeline")),
        {{yasmin_ros::basic_outcomes::SUCCEED, "STOP_WM"},
         {yasmin_ros::basic_outcomes::CANCEL, "STOP_WM"}});

    sm->add_state("STOP_WM",
                  std::make_shared<StopWaypointManagerState>(blackboard),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "DONE"}});

    sm->add_state(
        "DONE",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED},
            [](auto) {
                YASMIN_LOG_INFO("Pipeline inspection mission completed");
                return yasmin_ros::basic_outcomes::SUCCEED;
            }),
        {{yasmin_ros::basic_outcomes::SUCCEED,
          yasmin_ros::basic_outcomes::SUCCEED}});

    yasmin_viewer::YasminViewerPub viewer(sm, "PIPELINE_INSPECTION_FSM");

    rclcpp::on_shutdown([sm]() {
        if (sm->is_running())
            sm->cancel_state();
    });

    std::string outcome;

    try {
        outcome = (*sm)(blackboard);

        YASMIN_LOG_INFO("FSM finished with outcome: %s", outcome.c_str());
    } catch (const std::exception& e) {
        YASMIN_LOG_WARN(e.what());
        rcutils_reset_error();
    }

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();
        rclcpp::shutdown();
    }

    return 0;
}
