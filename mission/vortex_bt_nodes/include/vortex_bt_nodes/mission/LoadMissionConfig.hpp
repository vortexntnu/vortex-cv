#ifndef VORTEX_BT_NODES__MISSION__LOAD_MISSION_CONFIG_HPP_
#define VORTEX_BT_NODES__MISSION__LOAD_MISSION_CONFIG_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace vortex_bt_nodes::mission{
    class LoadMissionConfig: public BT::SyncActionNode{
        public: 
        LoadMissionConfig(const std::string& name,
            const BT::NodeConfig& config,
            rclcpp::Node::SharedPtr node
         );
        static BT::PortsList providedPorts();

        private: 
        BT::NodeStatus tick() override; 


    };




}


#endif 