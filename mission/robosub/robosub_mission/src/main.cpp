#include <behaviortree_cpp/bt_factory.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vortex_bt_nodes/register_nodes.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robosub_mission");

    const std::string default_tree =
        ament_index_cpp::get_package_share_directory("robosub_mission") +
        "/trees/root.xml";
    const std::string tree_file =
        node->declare_parameter<std::string>("tree_file", default_tree);
    const double tick_rate_hz =
        node->declare_parameter<double>("tick_rate_hz", 10.0);

    BT::BehaviorTreeFactory factory;
    vortex_bt_nodes::register_nodes(factory, node);

    spdlog::info("Starting RoboSub mission tree from {}", tree_file);
    BT::Tree tree;
    try {
        // root.xml includes the task files relative to itself.
        tree = factory.createTreeFromFile(tree_file);
    } catch (const std::exception& e) {
        spdlog::error("Could not load the tree: {}", e.what());
        rclcpp::shutdown();
        return 1;
    }

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(tick_rate_hz, 1.0));
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        rclcpp::spin_some(node);
        status = tree.tickOnce();
        std::this_thread::sleep_for(period);
    }
    tree.haltTree();

    spdlog::info("Mission tree finished: {}", BT::toStr(status));
    rclcpp::shutdown();
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
