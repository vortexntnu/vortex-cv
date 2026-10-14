#include "vortex_bt_nodes/map/go_to_frame_node.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace vortex_bt_nodes::map {

namespace {

constexpr char kBaseFrame[] = "base_link";

double distance(const geometry_msgs::msg::Point& a,
                const geometry_msgs::msg::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

GoToFrame::GoToFrame(const std::string& name,
                     const BT::NodeConfig& config,
                     rclcpp::Node::SharedPtr node,
                     std::shared_ptr<const LandmarkMap> map)
    : NavAction(name, config, node), map_(std::move(map)) {}

BT::PortsList GoToFrame::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::string>("frame", "TF frame to drive to"),
         BT::InputPort<Pose>("offset", "0;0;0",
                             "Offset in the frame's own axes [m], yaw [deg]"),
         BT::InputPort<std::string>("tool_frame", "",
                                    "Frame put on the target instead of "
                                    "base_link, empty = base_link"),
         BT::InputPort<std::string>("mode", "POSITION_AND_YAW",
                                    "WaypointMode name"),
         BT::InputPort<double>("resend_m", 0.1,
                               "[m] Send a new goal when the frame moved "
                               "further than this in odom"),
         BT::InputPort<double>("dead_reckoning_threshold", 1.0,
                               "[m] Stop following the frame closer than "
                               "this and drive on the last target"),
         BT::InputPort<double>("max_step_m", 3.0,
                               "[m] Split a longer move into several "
                               "waypoints, 0 = no split")});
}

std::optional<Pose> GoToFrame::target_in_odom() const {
    const auto frame = getInput<std::string>("frame");
    const auto offset = getInput<Pose>("offset");
    if (!frame || !offset) {
        return std::nullopt;
    }
    // The controller works in odom, so ask for the frame there and not in
    // map: a frame fixed in map moves whenever the map corrects drift.
    const auto frame_in_odom =
        map_->lookup(map_->odom_frame(), map_->frame_name(*frame));
    if (!frame_in_odom) {
        return std::nullopt;
    }
    Pose target = LandmarkMap::compose(*frame_in_odom, *offset);

    const auto tool = getInput<std::string>("tool_frame");
    if (tool && !tool->empty()) {
        // Put tool_frame on the target instead of base_link:
        // T_odom_base = T_odom_tool * T_tool_base.
        const auto base_in_tool =
            map_->lookup(map_->frame_name(*tool), map_->frame_name(kBaseFrame));
        if (!base_in_tool) {
            return std::nullopt;
        }
        target = LandmarkMap::compose(target, *base_in_tool);
    }
    return target;
}

std::optional<GoToFrame::Goal> GoToFrame::build_goal(
    const Pose& target) const {
    const auto mode_name = getInput<std::string>("mode");
    const auto mode = waypoint_mode_from_string(
        mode_name ? *mode_name : std::string("POSITION_AND_YAW"));
    if (!mode) {
        spdlog::error("[{}] unknown mode: {}", name(),
                      mode_name ? *mode_name : "");
        return std::nullopt;
    }

    const double max_step = getInput<double>("max_step_m").value_or(0.0);
    const auto vehicle = map_->vehicle_in_odom();
    int steps = 1;
    if (max_step > 0.0 && vehicle) {
        steps = std::max(
            1, static_cast<int>(
                   std::ceil(distance(vehicle->position, target.position) /
                             max_step)));
    }

    Goal goal;
    for (int i = 1; i <= steps; ++i) {
        Pose pose = target;
        if (vehicle && steps > 1) {
            const double t = static_cast<double>(i) / steps;
            pose.position.x = vehicle->position.x +
                              t * (target.position.x - vehicle->position.x);
            pose.position.y = vehicle->position.y +
                              t * (target.position.y - vehicle->position.y);
            pose.position.z = vehicle->position.z +
                              t * (target.position.z - vehicle->position.z);
        }
        goal.waypoints.push_back(make_waypoint(pose, mode->mode));
    }
    return goal;
}

std::optional<GoToFrame::Goal> GoToFrame::make_goal() {
    frozen_ = false;
    lost_ = false;

    const auto frame = getInput<std::string>("frame");
    if (!frame) {
        spdlog::error("[{}] {}", name(), frame.error());
        return std::nullopt;
    }
    const auto target = target_in_odom();
    if (!target) {
        spdlog::warn("[{}] {} is not in TF", name(), map_->frame_name(*frame));
        return std::nullopt;
    }
    last_target_ = *target;
    return build_goal(*target);
}

std::optional<GoToFrame::Goal> GoToFrame::update_goal() {
    if (frozen_) {
        return std::nullopt;
    }
    const auto frame = getInput<std::string>("frame");
    if (!frame) {
        return std::nullopt;
    }

    // Close up the detections are poor: keep the last target and drive on.
    const auto vehicle = map_->vehicle_in_odom();
    const double dead_reckoning =
        getInput<double>("dead_reckoning_threshold").value_or(1.0);
    if (vehicle && distance(vehicle->position, last_target_.position) <
                      dead_reckoning) {
        frozen_ = true;
        spdlog::info("[{}] inside {} m of {}, holding the last target", name(),
                     dead_reckoning, map_->frame_name(*frame));
        return std::nullopt;
    }

    const auto target = target_in_odom();
    if (!target) {
        if (!lost_) {
            spdlog::warn("[{}] {} left TF, driving on the last target", name(),
                         map_->frame_name(*frame));
            lost_ = true;
        }
        return std::nullopt;
    }
    lost_ = false;

    const double resend_m = getInput<double>("resend_m").value_or(0.1);
    if (distance(last_target_.position, target->position) < resend_m) {
        return std::nullopt;
    }
    last_target_ = *target;
    return build_goal(*target);
}

}  // namespace vortex_bt_nodes::map
