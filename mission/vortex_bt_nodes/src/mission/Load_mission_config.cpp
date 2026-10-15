#include "vortex_bt_nodes/mission/Load_mission_config.hpp"
#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <vector>


namespace vortex_bt_nodes::mission{
    void write_yaml_value(BT::Blackboard::Ptr blackboard,
    const std::string& key, 
    const YAML::Node& node){
        blackboard->set(key, node.as<std::string>());
    }

    void flatten_yaml(BT::Blackboard::Ptr blackboard,
                    const YAML::Node& node, 
                    const std::string& prefix=""){
        
        if(node.IsMap()){
            for(const auto& entry:node){
            const auto key=entry.first.as<std::string>();
            const auto next=prefix.empty() ? key: prefix + "."+key;
            flatten_yaml(blackboard, entry.second, next); 
            }
            return;
        }

        if (node.IsSequence()) {
            std::string value;
            for (std::size_t i = 0; i < node.size(); ++i) {
            if (i != 0) value += ",";
                value += node[i].as<std::string>();
    }
    blackboard->set(prefix, value);
    return;
    }

        if (node.IsScalar()) {
        blackboard->set(prefix, node.as<std::string>());
        }

}





    LoadMissionConfig::LoadMissionConfig(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node)
        :BT::SyncActionNode(name,config), node_(std::move(node)) {}

        BT::PortsList LoadMissionConfig::providedPorts() {
            return{BT::InputPort<std::string>("path","Path to the mission yaml file")};
        }

        BT::NodeStatus LoadMissionConfig::tick(){
            auto path=getInput<std::string>("path");
            if(!path){
                spdlog::error("Feilet, Kunne ikke lese path til YAML filen");
                return BT::NodeStatus::FAILURE;

            }
            
            const std::filesystem::path yaml_path(*path);
            if(!std::filesystem::exists(yaml_path)){
                spdlog::error("Filen finnes ikke");
                return BT::NodeStatus::FAILURE;
            }


            const YAML::Node doc = YAML::LoadFile(yaml_path.string());
            auto blackboard = config().blackboard;
            if (!blackboard) {
               spdlog::error("Greide ikke å koble seg til Blackboard");
                return BT::NodeStatus::FAILURE;
            } 

            flatten_yaml(blackboard,doc);
            return BT::NodeStatus::SUCCESS;
        }

}