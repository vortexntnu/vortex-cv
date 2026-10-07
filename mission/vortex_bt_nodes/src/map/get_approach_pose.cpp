#include "vortex_bt_nodes/map/get_approach_pose.hpp"

#include <spdlog/spdlog.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace vortex_bt_nodes::map {

BT::PortsList GetApproachPose::providedPorts() {
    BT::PortsList ports = LandmarkCache::ports();
    ports.insert(BT::InputPort<Pose>(
        "offset", "x;y;z from the landmark, landmark frame (+X out of front)"));
    ports.insert(BT::InputPort<int>(
        "symmetry_deg", 0, "Yaw symmetry of the object (0, 90, 180, 360)"));
    ports.insert(BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Approach pose (map frame)"));
    return ports;
}

BT::NodeStatus GetApproachPose::tick() {
    const auto landmark = cache_->resolve(*this);
    const auto offset = getInput<Pose>("offset");
    const auto vehicle = cache_->vehicle_pose();
    if (!landmark || !offset || !vehicle) {
        spdlog::warn("[{}] {}", name(),
                     !landmark ? "landmark not in the map"
                     : !offset ? "bad offset"
                               : "no vehicle pose in the map frame (TF)");
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = cache_->frame_id();
    out.pose =
        LandmarkCache::approach_pose(*landmark, *vehicle, *offset,
                                     getInput<int>("symmetry_deg").value_or(0));
    setOutput("pose", out);
    spdlog::info(
        "[{}] landmark {}: approach [{:.2f}, {:.2f}, {:.2f}] yaw {:.0f} deg",
        name(), landmark->id, out.pose.position.x, out.pose.position.y,
        out.pose.position.z, yaw_of(out.pose) * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace vortex_bt_nodes::map
