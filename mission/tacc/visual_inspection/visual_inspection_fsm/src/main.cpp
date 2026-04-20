#include "visual_inspection_fsm/states.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    yasmin_ros::set_ros_loggers();

    YASMIN_LOG_INFO("Starting Visual Inspection FSM");

    auto node = rclcpp::Node::make_shared("visual_inspection_fsm");

    const auto config = load_config(node);
    auto blackboard = initialize_blackboard(config);

    // --- TF2 for drone position lookups ---
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);

    // Derive TF frame names from the node namespace (e.g. "/moby" → "moby")
    std::string ns = node->get_namespace();
    if (!ns.empty() && ns[0] == '/') {
        ns = ns.substr(1);
    }
    const std::string odom_frame = ns.empty() ? "odom" : ns + "/odom";
    const std::string base_link_frame =
        ns.empty() ? "base_link" : ns + "/base_link";

    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer);
    blackboard->set<std::string>("odom_frame", odom_frame);
    blackboard->set<std::string>("base_link_frame", base_link_frame);

    YASMIN_LOG_INFO("TF frames: odom=%s, base_link=%s", odom_frame.c_str(),
                    base_link_frame.c_str());

    auto sm = build_state_machine(config, blackboard);

    yasmin_viewer::YasminViewerPub viewer(sm, "VISUAL_INSPECTION_FSM");

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
