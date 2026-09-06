#include "learning_period/states.hpp"

#include <memory>

#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    yasmin_ros::set_ros_loggers();

    YASMIN_LOG_INFO("Starting Learning Period FSM");

    auto node = rclcpp::Node::make_shared("learning_period_fsm");

    const auto config = load_config(node);
    auto blackboard = initialize_blackboard(config);
    auto sm = build_state_machine(config, blackboard);

    // Heap-allocated so it can be torn down explicitly (joining its background
    // publisher thread) while the ROS context is still valid — before the
    // rclcpp::shutdown() below. A stack local would be destroyed at return,
    // after shutdown, and its thread would segfault publishing on a dead
    // context.
    auto viewer = std::make_unique<yasmin_viewer::YasminViewerPub>(
        sm, "LEARNING_PERIOD_FSM");

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

    viewer.reset();

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();
        rclcpp::shutdown();
    }

    return 0;
}
