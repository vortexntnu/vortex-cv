#ifndef VORTEX_BT_NODES__MAP__COMMIT_TARGET_HPP_
#define VORTEX_BT_NODES__MAP__COMMIT_TARGET_HPP_

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <optional>
#include <string>

#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

/** @brief Snapshot a target in odom once its position stays within stable_m. */
class CommitTarget : public BT::StatefulActionNode {
   public:
    CommitTarget(const std::string& name,
                 const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node,
                 std::shared_ptr<const LandmarkMap> map);

    static BT::PortsList providedPorts();

   private:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<const LandmarkMap> map_;
    std::string frame_;
    double stable_m_{0.0};
    double stable_s_{0.0};
    std::optional<Pose> anchor_;
    rclcpp::Time stable_since_;
    rclcpp::Time last_tick_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__COMMIT_TARGET_HPP_
