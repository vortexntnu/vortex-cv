#ifndef VORTEX_BT_NODES__MISSION__START_RUN_HPP_
#define VORTEX_BT_NODES__MISSION__START_RUN_HPP_

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace vortex_bt_nodes::mission {

/**
 * @brief Starts the run: publishes mission/wipe (map, waypoint_manager and
 * reference filter start over) and gives landmark_slam the coin flip
 * (start_yaw_offset_deg: the map is anchored at the current pose). FAILURE
 * if landmark_slam does not take the parameter within timeout_s.
 */
class StartRun : public BT::StatefulActionNode {
   public:
    StartRun(const std::string& name,
             const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

   private:
    using Results = std::vector<rcl_interfaces::msg::SetParametersResult>;

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr wipe_pub_;
    std::shared_ptr<rclcpp::AsyncParametersClient> params_;
    std::optional<std::shared_future<Results>> pending_;
    rclcpp::Time start_;
};

}  // namespace vortex_bt_nodes::mission

#endif  // VORTEX_BT_NODES__MISSION__START_RUN_HPP_
