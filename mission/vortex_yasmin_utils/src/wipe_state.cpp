#include "vortex_yasmin_utils/wipe_state.hpp"

#include <vortex/utils/ros/qos_profiles.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/yasmin_node.hpp>

namespace vortex_yasmin_utils {

WipeState::WipeState() : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {
    auto node = yasmin_ros::YasminNode::get_instance();
    pub_ = node->create_publisher<std_msgs::msg::Empty>(
        "mission/wipe", vortex::utils::qos_profiles::reliable_profile(1));
}

std::string WipeState::execute(yasmin::Blackboard::SharedPtr) {
    YASMIN_LOG_INFO(
        "Wiping mission state (reference filter, landmark server, waypoint "
        "manager)");
    pub_->publish(std_msgs::msg::Empty{});
    return yasmin_ros::basic_outcomes::SUCCEED;
}

}  // namespace vortex_yasmin_utils
