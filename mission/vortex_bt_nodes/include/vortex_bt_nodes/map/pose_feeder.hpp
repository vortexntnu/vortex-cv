#ifndef VORTEX_BT_NODES__MAP__POSE_FEEDER_HPP_
#define VORTEX_BT_NODES__MAP__POSE_FEEDER_HPP_

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "vortex_bt_nodes/common/types.hpp"

namespace vortex_bt_nodes::map {

    class PoseFeeder : public BT::SyncActionNode {
        public:
            using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

            PoseFeeder( const std::string& name, 
                        const BT::NodeConfig& config,
                        rclcpp::Node::SharedPtr node);

            static BT::PortsList providedPorts();

        private:
            BT::NodeStatus tick() override;

            void pose_callback(const PoseMSG::SharedPtr msg);

            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<PoseMSG>::SharedPtr sub_;
            std::string topic_;
            std::optional<PoseMSG> latest_;
};
}

#endif  // VORTEX_BT_NODES__MAP__POSE_FEEDER_HPP_