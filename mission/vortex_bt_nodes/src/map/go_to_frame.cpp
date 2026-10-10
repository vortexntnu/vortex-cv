#include "vortex_bt_nodes/map/go_to_frame.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

namespace vortex_bt_nodes::map {

namespace {

double distance(const Pose& a, const Pose& b) {
    return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y,
                      a.position.z - b.position.z);
}

}  // namespace

BT::PortsList GoToFrame::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::string>(
             "frame", "TF frame, e.g. gate_search_rescue_entrance"),
         BT::InputPort<Pose>("offset", "0;0;0", "x;y;z[;yaw_deg] in the frame"),
         BT::InputPort<std::string>("mode", "POSITION_AND_YAW",
                                    "WaypointMode name"),
         BT::InputPort<double>("resend_m", 0.1,
                               "Send the goal again when the target moved "
                               "this much [m]"),
         BT::InputPort<double>("freeze_within_m", 1.0,
                               "Stop updating the target this close [m]"),
         BT::InputPort<double>("max_step_m", 2.0,
                               "Longer moves get waypoints this far apart "
                               "[m], 0 = one waypoint")});
}

std::optional<Pose> GoToFrame::target() const {
    const auto frame = getInput<std::string>("frame");
    const auto offset = getInput<Pose>("offset");
    if (!frame || !offset) {
        return std::nullopt;
    }
    const auto in_odom =
        cache_->lookup(cache_->odom_frame(), cache_->frame_name(*frame));
    if (!in_odom) {
        return std::nullopt;
    }
    return LandmarkCache::compose(*in_odom, *offset);
}

std::optional<NavAction::Goal> GoToFrame::goal_for(const Pose& target) const {
    const auto mode = waypoint_mode_from_string(
        getInput<std::string>("mode").value_or("POSITION_AND_YAW"));
    if (!mode) {
        spdlog::warn("[{}] unknown mode", name());
        return std::nullopt;
    }
    vortex_msgs::msg::Waypoint wp;
    wp.pose = target;
    wp.waypoint_mode = *mode;
    Goal goal;
    // One long step can saturate the thrusters and flip the vehicle.
    const double step = getInput<double>("max_step_m").value_or(2.0);
    const auto vehicle = cache_->vehicle_in_odom();
    if (step > 0.0 && vehicle) {
        const int n = static_cast<int>(std::ceil(distance(*vehicle, target) / step));
        for (int i = 1; i < n; ++i) {
            const double a = static_cast<double>(i) / n;
            auto via = wp;
            via.pose.position.x = std::lerp(vehicle->position.x, target.position.x, a);
            via.pose.position.y = std::lerp(vehicle->position.y, target.position.y, a);
            via.pose.position.z = std::lerp(vehicle->position.z, target.position.z, a);
            via.position_tolerance = 0.5;
            via.hold_time_sec = 0.01;
            goal.waypoints.push_back(via);
        }
    }
    goal.waypoints.push_back(wp);
    goal.convergence_threshold = 0.1;
    return goal;
}

std::optional<NavAction::Goal> GoToFrame::make_goal() {
    frozen_ = false;
    const auto t = target();
    if (!t) {
        spdlog::warn("[{}] frame '{}' not in TF", name(),
                     getInput<std::string>("frame").value_or(""));
        return std::nullopt;
    }
    sent_target_ = *t;
    spdlog::info("[{}] target [{:.2f}, {:.2f}, {:.2f}] (odom)", name(),
                 t->position.x, t->position.y, t->position.z);
    return goal_for(*t);
}

std::optional<NavAction::Goal> GoToFrame::update_goal() {
    if (frozen_) {
        return std::nullopt;
    }
    const auto t = target();
    if (!t) {
        return std::nullopt;  // keep the last goal
    }
    const auto vehicle = cache_->vehicle_in_odom();
    if (vehicle && distance(*vehicle, *t) <
                       getInput<double>("freeze_within_m").value_or(1.0)) {
        frozen_ = true;
        spdlog::info("[{}] within freeze distance, target fixed", name());
        return std::nullopt;
    }
    if (distance(*t, sent_target_) <
        getInput<double>("resend_m").value_or(0.1)) {
        return std::nullopt;
    }
    spdlog::info(
        "[{}] target moved {:.2f} m, new goal [{:.2f}, {:.2f}, {:.2f}]", name(),
        distance(*t, sent_target_), t->position.x, t->position.y,
        t->position.z);
    sent_target_ = *t;
    return goal_for(*t);
}

}  // namespace vortex_bt_nodes::map
