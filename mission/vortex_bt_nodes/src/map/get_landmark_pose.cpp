#include "vortex_bt_nodes/map/get_landmark_pose.hpp"

#include <spdlog/spdlog.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace vortex_bt_nodes::map {

BT::PortsList GetLandmarkPose::providedPorts() {
    BT::PortsList ports = LandmarkCache::ports();
    ports.insert(BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Landmark pose in odom (drift-corrected)"));
    return ports;
}

BT::NodeStatus GetLandmarkPose::tick() {
    const auto landmark = cache_->resolve(*this);
    if (!landmark) {
        spdlog::warn("[{}] landmark not in the map", name());
        return BT::NodeStatus::FAILURE;
    }
    const auto in_odom = cache_->map_to_odom(landmark->pose);
    if (!in_odom) {
        spdlog::warn("[{}] no TF map -> odom", name());
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = cache_->odom_frame();
    out.pose = *in_odom;
    setOutput("pose", out);
    spdlog::info("[{}] landmark {}: [{:.2f}, {:.2f}, {:.2f}] yaw {:.0f} deg",
                 name(), landmark->id, out.pose.position.x, out.pose.position.y,
                 out.pose.position.z, yaw_of(out.pose) * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace vortex_bt_nodes::map
