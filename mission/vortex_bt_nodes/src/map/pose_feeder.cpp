#include "vortex_bt_nodes/pose_feeder.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::map {
    PoseFeeder::PoseFeeder(const std::string& name,
                           const BT::NodeConfig& config,
                           rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {}

    BT::PortsList PoseFeeder::providedPorts() {
        return{
            BT::InputPort<std::string>("topic", "pose",
                    "PoseWithCovarianceStamped topic"),
            BT::InputPort<double>("max_age_s", 0.5, "[s], older messages fail the node"),
            BT::OutputPort<Pose>("pose", "Vehicle pose in map frame"),
            };
        }

    BT::NodeStatus PoseFeeder::tick() {
        const auto topic = getInput<std::string>("topic");
        if(!topic){
            spdlog::error("[{}] bad topic port: {}", name(), topic.error());
            return BT::NodeStatus::FAILURE;
        }
        if(!sub_ || *topic != topic_){
            topic_ = *topic;
            latest_.reset();
            sub_ = node_->create_subscription<PoseMSG>(
                topic_, rclcpp::SensorDataQoS(),
                [this](const PoseMSG::SharedPtr msg){pose_callback(msg);});
        }

        if(!latest:){
            spdlog::warn("[{}] no pose received on {}", name(), topic:_);
            return BT::NodeStatus::FAILURE;
        }

        const auto max_age_s = getInput<double>("max_age_s");
        if(!max_age_s){
            spdlog::error("[{}] bad max_age_s port: {}", name(), max_age_s.error());
            return BT::NodeStatus::FAILURE;
        }

        const double age_s = (node_->now() - rclcpp::Time(latest_->header.stamp)).seconds()
        if(age_s > *max_age_s){
            spdlog::warn("[{}] pose is {:.2f} s old (max {:.2f} s)", name(), age_s,
                     *max_age_s);
            return BT::NodeStatus::FAILURE;
        }

        setOutput("pose", latest_->pose.pose);
        return BT::NodeStatus::SUCCESS;

    }

    void PoseFeeder::pose_callback(const PoseMSG::SharedPtr msg){
        latest_ = *msg;
    }
}