#include "vortex_bt_nodes/motion/search.hpp"

#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <cmath>

namespace vortex_bt_nodes::motion {

Search::Search(const std::string& name,
               const BT::NodeConfig& config,
               rclcpp::Node::SharedPtr node,
               std::shared_ptr<const map::LandmarkMap> map)
    : NavAction(name, config, std::move(node)), map_(std::move(map)) {}

BT::PortsList Search::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("pattern", "ROTATE_STEPS or SCAN_ARC"),
        BT::InputPort<double>("step_deg", "Heading step [deg], 0 < step < 180"),
        BT::InputPort<double>("arc_deg", "SCAN_ARC width [deg], 0 < arc <= 360"),
        BT::InputPort<double>("pause_s", "Pause at each heading [s], nonnegative")});
}

std::optional<Search::Goal> Search::make_goal() {
    const auto pattern = getInput<std::string>("pattern");
    const auto step = getInput<double>("step_deg");
    const auto pause = getInput<double>("pause_s");
    if (!pattern || (*pattern != "ROTATE_STEPS" && *pattern != "SCAN_ARC") ||
        !step || !std::isfinite(*step) || *step <= 0.0 || *step >= 180.0 ||
        !pause || !std::isfinite(*pause) || *pause < 0.0) {
        spdlog::error("[{}] invalid or missing search pattern, step_deg or pause_s", name());
        return std::nullopt;
    }
    double width = 360.0;
    const bool scan = *pattern == "SCAN_ARC";
    if (scan) {
        const auto arc = getInput<double>("arc_deg");
        if (!arc || !std::isfinite(*arc) || *arc <= 0.0 || *arc > 360.0) {
            spdlog::error("[{}] arc_deg must be finite and in (0, 360]", name());
            return std::nullopt;
        }
        width = *arc;
    }
    // Bound allocation and reject steps too small for a useful mission sweep.
    if (std::ceil(width / *step) + (scan ? std::ceil(width / (2.0 * *step)) : 0.0) > 10000.0) {
        spdlog::error("[{}] search requires more than 10000 waypoints", name());
        return std::nullopt;
    }
    const auto vehicle = map_->vehicle_in_odom();
    if (!vehicle) {
        spdlog::warn("[{}] vehicle pose is not in TF", name());
        return std::nullopt;
    }
    tf2::Quaternion orientation;
    tf2::fromMsg(vehicle->orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    Goal goal;
    const auto append_segment = [&](double from, double to) {
        const int count = static_cast<int>(std::ceil(std::abs(to - from) / *step));
        for (int i = 1; i <= count; ++i) {
            const double offset = from + std::copysign(
                std::min(i * *step, std::abs(to - from)), to - from);
            Pose pose = *vehicle;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw + offset * M_PI / 180.0);
            pose.orientation = tf2::toMsg(q);
            auto wp = make_waypoint(pose, vortex_msgs::msg::WaypointMode::FULL_POSE);
            wp.hold_time_sec = *pause;
            goal.waypoints.push_back(wp);
        }
    };
    if (scan) {
        append_segment(0.0, -width / 2.0);
        append_segment(-width / 2.0, width / 2.0);
    } else {
        append_segment(0.0, width);
    }
    return goal;
}

BT::NodeStatus Search::on_result(const GoalHandle::WrappedResult& result) {
    if (NavAction::on_result(result) == BT::NodeStatus::SUCCESS) {
        spdlog::info("[{}] search sweep exhausted", name());
    }
    return BT::NodeStatus::FAILURE;
}

}  // namespace vortex_bt_nodes::motion
