#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <memory>
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
    const std::string main_tree =
        node->declare_parameter<std::string>("main_tree", "Main");
    const std::string mission_config =
        node->declare_parameter<std::string>("mission_config", "");
    const double tick_rate_hz =
        node->declare_parameter<double>("tick_rate_hz", 10.0);
    const int groot_port =
        static_cast<int>(node->declare_parameter<int>("groot_port", 1666));

    BT::BehaviorTreeFactory factory;
    vortex_bt_nodes::register_nodes(factory, node);

    spdlog::info("Starting RoboSub mission tree {} from {}", main_tree,
                 tree_file);
    BT::Tree tree;
    try {
        factory.registerBehaviorTreeFromFile(tree_file);
        auto blackboard = BT::Blackboard::create();
        blackboard->set("mission_config", mission_config);
        tree = factory.createTree(main_tree, blackboard);
    } catch (const std::exception& e) {
        spdlog::error("Could not load the tree: {}", e.what());
        rclcpp::shutdown();
        return 1;
    }
    // Groot2 or the VS Code BehaviorTree Viewer. 0 = off.
    std::unique_ptr<BT::Groot2Publisher> groot;
    if (groot_port > 0) {
        try {
            groot = std::make_unique<BT::Groot2Publisher>(tree, groot_port);
        } catch (const std::exception& e) {
            spdlog::warn("No Groot2 view on port {}: {}", groot_port, e.what());
        }
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
