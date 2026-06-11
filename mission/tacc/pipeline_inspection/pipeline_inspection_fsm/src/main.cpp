#include "pipeline_inspection_fsm/states.hpp"

#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    yasmin_ros::set_ros_loggers();

    YASMIN_LOG_INFO("Starting Pipeline Inspection FSM");

    auto node = rclcpp::Node::make_shared("pipeline_inspection_fsm");

    const auto config = load_config(node);
    auto blackboard = initialize_blackboard(config);

    // Capture the drone pose continuously so STORE_ORIGIN can snapshot it.
    auto latest_pose = std::make_shared<std::optional<geometry_msgs::msg::Pose>>();
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        config.odom_topic, rclcpp::QoS(1).best_effort(),
        [latest_pose](nav_msgs::msg::Odometry::SharedPtr msg) {
            *latest_pose = msg->pose.pose;
        });

    // Spin node in background so odom callbacks are processed while FSM runs.
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    auto spin_thread = std::thread([executor]() { executor->spin(); });

    auto sm = build_state_machine(config, blackboard, latest_pose);

    yasmin_viewer::YasminViewerPub viewer(sm, "PIPELINE_INSPECTION_FSM");

    rclcpp::on_shutdown([sm, executor]() {
        if (sm->is_running())
            sm->cancel_state();
        executor->cancel();
    });

    std::string outcome;
    try {
        outcome = (*sm)(blackboard);
        YASMIN_LOG_INFO("FSM finished with outcome: %s", outcome.c_str());
    } catch (const std::exception& e) {
        YASMIN_LOG_WARN(e.what());
        rcutils_reset_error();
    }

    executor->cancel();
    if (spin_thread.joinable())
        spin_thread.join();

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();
        rclcpp::shutdown();
    }

    return 0;
}
