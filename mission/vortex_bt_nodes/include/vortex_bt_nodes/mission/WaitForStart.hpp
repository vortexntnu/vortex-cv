
#ifndef VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_
#define VORTEX_BT_NODES__MISSION__WAIT_FOR_START_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <cstdlib>
#include <vortex_msgs/msg/operation_mode.hpp>
#include <vortex_msgs/srv/get_operation_mode.hpp>


namespace vortex_bt_nodes::mission {

        class WaitForStart : public BT::StatefulActionNode{
            public: 
            WaitForStart(const std::string& name, 
                const BT::NodeConfig& config, 
                rclcpp::Node::SharedPtr node);
        

        static BT::PortsList providedPorts();

        private: 
        using Service=vortex_msgs::srv::GetOperationMode;
        using Client=rclcpp::Client<Service>;
        using Clock=std::chrono::steady_clock;


        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;


        rclcpp::Node::SharedPtr node_;
        Client::SharedPtr client_;
        std::optional<Client::FutureAndRequestId> future_;


        Clock::time_point next_request_{};
        Clock::time_point request_deadline_{};

        std::chrono::seconds request_interval_{5};
        std::chrono::seconds request_timeout_{5};
    };
}  

#endif 
