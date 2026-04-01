#include "valve_inspection_fsm/states.hpp"
#include "valve_inspection_fsm/valve_converge_state.hpp"

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <yasmin_viewer/yasmin_viewer_pub.hpp>

#include <rclcpp/rclcpp.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    yasmin_ros::set_ros_loggers();

    YASMIN_LOG_INFO("Starting Valve Inspection FSM");

    auto blackboard = initialize_blackboard();

    vortex_msgs::msg::LandmarkType valve_type;
    valve_type.value = vortex_msgs::msg::LandmarkType::VALVE;

    vortex_msgs::msg::LandmarkSubtype valve_subtype;
    valve_subtype.value = vortex_msgs::msg::LandmarkSubtype::VALVE_VERTICAL;

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                              yasmin_ros::basic_outcomes::ABORT});

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            blackboard->get<std::string>("service.start_mission")),
        {{yasmin_ros::basic_outcomes::SUCCEED, "POLL_VALVE"},
         {yasmin_ros::basic_outcomes::CANCEL,
          yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state(
        "POLL_VALVE",
        std::make_shared<vortex_yasmin_utils::LandmarkPollingState>(
            blackboard->get<std::string>("action_server.landmark_polling"),
            valve_type, valve_subtype, "valve_landmarks", "valve_found"),
        {{"valve_found", "CONVERGE_VALVE"},
         {yasmin_ros::basic_outcomes::ABORT,
          yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state(
        "CONVERGE_VALVE",
        std::make_shared<valve_inspection_fsm::ValveConvergeState>(
            blackboard->get<std::string>("action_server.landmark_convergence"),
            valve_type, valve_subtype),
        {{yasmin_ros::basic_outcomes::SUCCEED, "DONE"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
         {yasmin_ros::basic_outcomes::CANCEL,
          yasmin_ros::basic_outcomes::ABORT}});

    sm->add_state("DONE",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED},
                      [](auto) {
                          YASMIN_LOG_INFO("Valve inspection mission completed");
                          return yasmin_ros::basic_outcomes::SUCCEED;
                      }),
                  {{yasmin_ros::basic_outcomes::SUCCEED,
                    yasmin_ros::basic_outcomes::SUCCEED}});

    yasmin_viewer::YasminViewerPub viewer(sm, "VALVE_INSPECTION_FSM");

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
