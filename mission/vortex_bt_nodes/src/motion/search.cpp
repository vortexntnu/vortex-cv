#include "vortex_bt_nodes/motion/search.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>

#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex_bt_nodes::motion {

namespace {
constexpr double kHeadingToleranceDeg = 10.0;
}  // namespace

BT::PortsList Search::providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::string>("pattern", "ROTATE_STEPS",
                                    "ROTATE_STEPS or SCAN_ARC"),
         BT::InputPort<double>("step_deg", 45.0, "ROTATE_STEPS step"),
         BT::InputPort<double>("arc_deg", 60.0, "SCAN_ARC: to each side"),
         BT::InputPort<double>("pause_s", 1.5, "Hold at each heading")});
}

std::optional<NavAction::Goal> Search::make_goal() {
    const std::string pattern =
        getInput<std::string>("pattern").value_or("ROTATE_STEPS");
    std::vector<double> headings_deg;
    if (pattern == "ROTATE_STEPS") {
        const double step =
            std::abs(getInput<double>("step_deg").value_or(45.0));
        if (step < 1.0) {
            spdlog::warn("[{}] step_deg too small", name());
            return std::nullopt;
        }
        for (double h = step; h <= 360.0 + 1e-6; h += step) {
            headings_deg.push_back(h);
        }
    } else if (pattern == "SCAN_ARC") {
        const double arc = std::abs(getInput<double>("arc_deg").value_or(60.0));
        headings_deg = {arc, -arc, 0.0};
    } else {
        spdlog::warn("[{}] unknown pattern '{}'", name(), pattern);
        return std::nullopt;
    }
    const double pause = getInput<double>("pause_s").value_or(1.5);

    // Every heading is relative to the heading at the start.
    Goal goal;
    goal.frame = Goal::BODY_RELATIVE;
    goal.convergence_threshold = 0.1;
    for (const double h : headings_deg) {
        auto wp =
            make_waypoint(make_pose(0.0, 0.0, 0.0, h * M_PI / 180.0),
                          vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION);
        wp.orientation_tolerance = kHeadingToleranceDeg * M_PI / 180.0;
        wp.position_tolerance = 0.5;
        wp.hold_time_sec = pause;
        goal.waypoints.push_back(wp);
    }
    spdlog::info("[{}] {} with {} headings", name(), pattern,
                 headings_deg.size());
    return goal;
}

BT::NodeStatus Search::on_result(const GoalHandle::WrappedResult& result) {
    // Nothing stopped the sweep, so the target was not found.
    NavAction::on_result(result);
    spdlog::info("[{}] sweep done, nothing found", name());
    return BT::NodeStatus::FAILURE;
}

}  // namespace vortex_bt_nodes::motion
