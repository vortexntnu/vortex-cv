#include "subsea_docking_fsm/states.hpp"

#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    yasmin_ros::set_ros_loggers();

    spdlog::info("Starting Subsea Docking FSM");

    auto node = rclcpp::Node::make_shared("subsea_docking_fsm");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });

    const auto config = load_config(node);
    auto blackboard = initialize_blackboard(config);
    auto sm = build_state_machine(config, blackboard);

    yasmin_viewer::YasminViewerPub viewer(node, sm, "SUBSEA_DOCKING_FSM");

    rclcpp::on_shutdown([sm]() {
        if (sm->is_running())
            sm->cancel_state();
    });

    std::string outcome;
    try {
        outcome = (*sm)(blackboard);
        spdlog::info("FSM finished with outcome: {}", outcome);
    } catch (const std::exception& e) {
        spdlog::warn("Error occurred: {}", e.what());
        rcutils_reset_error();
    }

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();
        rclcpp::shutdown();
    }

    executor.cancel();
    spin_thread.join();

    return 0;
}
