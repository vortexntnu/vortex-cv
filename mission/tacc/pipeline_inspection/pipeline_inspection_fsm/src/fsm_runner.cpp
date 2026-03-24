#include "pipeline_inspection_fsm/states.hpp"

#include <yasmin/cb_state.hpp>
#include <yasmin/concurrence.hpp>
#include <yasmin/state_machine.hpp>

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/service_state.hpp>

#include <yasmin_viewer/yasmin_viewer_pub.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace {

std::string get_string_parameter(const rclcpp::Node::SharedPtr& node,
                                 const std::string& name) {
    if (!node->has_parameter(name))
        node->declare_parameter<std::string>(name);

    return node->get_parameter(name).as_string();
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    yasmin_ros::set_ros_loggers();

    YASMIN_LOG_INFO("Starting Pipeline Inspection FSM");

    auto blackboard = initialize_blackboard();

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                              yasmin_ros::basic_outcomes::ABORT});

    auto search = std::make_shared<yasmin::Concurrence>(
        yasmin::StateMap{{"SEARCH_PATTERN",
                          std::make_shared<SearchPatternState>(blackboard)},
                         {"LANDMARK_POLLING",
                          std::make_shared<LandmarkPollingState>(blackboard)}},
        yasmin_ros::basic_outcomes::ABORT,
        yasmin::OutcomeMap{
            {"landmark_found",
             yasmin::StateOutcomeMap{{"LANDMARK_POLLING", "landmark_found"}}}});

    auto start_wm = std::make_shared<StartWaypointManagerState>(blackboard);

    auto pipeline_following =
        std::make_shared<WaitForPipelineEndState>(blackboard);

    auto stop_wm = std::make_shared<StopWaypointManagerState>(blackboard);

    auto start_pipeline_trg =
        std::make_shared<yasmin_ros::ServiceState<pipeline_inspection_fsm::TriggerSrv>>(
            get_string_parameter(yasmin_ros::YasminNode::get_instance(),
                                 "services.start_pipeline_following"),
            [](yasmin::Blackboard::SharedPtr) {
                return std::make_shared<pipeline_inspection_fsm::TriggerSrv::Request>();
            });

    sm->add_state("WAIT_FOR_START",
                  std::make_shared<WaitForStartState>(blackboard),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "SEARCH"},
                   {yasmin_ros::basic_outcomes::CANCEL,
                    yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state("SEARCH", search,
                  {{"landmark_found", "CONVERGE"},
                   {yasmin_ros::basic_outcomes::ABORT,
                    yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state(
        "CONVERGE", std::make_shared<ConvergeState>(blackboard),
        {{yasmin_ros::basic_outcomes::SUCCEED, "START_PIPELINE_TRG"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
         {yasmin_ros::basic_outcomes::CANCEL,
          yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state("START_PIPELINE_TRG", start_pipeline_trg,
                  {{yasmin_ros::basic_outcomes::SUCCEED, "START_WM"},
                   {yasmin_ros::basic_outcomes::ABORT,
                    yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state("START_WM", start_wm,
                  {{yasmin_ros::basic_outcomes::SUCCEED, "PIPELINE_FOLLOWING"},
                   {yasmin_ros::basic_outcomes::ABORT,
                    yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state("PIPELINE_FOLLOWING", pipeline_following,
                  {{yasmin_ros::basic_outcomes::SUCCEED, "STOP_WM"},
                   {yasmin_ros::basic_outcomes::CANCEL, "STOP_WM"}});

    sm->add_state("STOP_WM", stop_wm,
                  {{yasmin_ros::basic_outcomes::SUCCEED, "DONE"}});

    sm->add_state("DONE",
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
