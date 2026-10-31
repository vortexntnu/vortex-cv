#include "vortex_bt_nodes/map/go_to_frame_node.hpp"

#include <spdlog/spdlog.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace vortex_bt_nodes::map {

namespace {

constexpr char kBaseFrame[] = "base_link";
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kMinLegM = 0.3;   // shorter than this: one straight move
constexpr double kFaceLegM = 1.5;  // longer than this: driven facing forward
// Waypoints on the way are passed, not stopped at: only the last one uses the
// reference filter's tight default tolerance.
constexpr double kPassPositionM = 0.4;
constexpr double kPassOrientationRad = 0.2;
// Closer than this and no longer getting closer: there. The vehicle can hold
// a steady 0.15 m off in depth.
constexpr double kSettleM = 0.2;
constexpr double kSettleDeg = 6.0;
// Getting this much closer to the target counts as progress.
constexpr double kProgressM = 0.03;
constexpr double kProgressDeg = 2.0;

double distance(const geometry_msgs::msg::Point& a,
                const geometry_msgs::msg::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double rotation_deg(const geometry_msgs::msg::Quaternion& a,
                    const geometry_msgs::msg::Quaternion& b) {
    tf2::Quaternion qa, qb;
    tf2::fromMsg(a, qa);
    tf2::fromMsg(b, qb);
    const tf2::Quaternion e = qa.inverse() * qb;
    const double vec = std::sqrt(e.x() * e.x() + e.y() * e.y() + e.z() * e.z());
    return 2.0 * std::atan2(vec, std::abs(e.w())) * kRad2Deg;
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
         BT::InputPort<std::string>("mode", "FULL_POSE", "WaypointMode name"),
         BT::InputPort<double>("resend_m", 0.1,
                               "[m] Send a new goal when the frame moved "
                               "further than this in odom"),
         BT::InputPort<double>("resend_deg", 10,
                               "[deg] Send a new goal when the frame rotated "
                               "further than this in odom"),

         BT::InputPort<double>("dead_reckoning_threshold", 1.0,
                               "[m] Stop following the frame closer than "
                               "this and drive on the last target"),
         BT::InputPort<double>("max_step_m", 3.0,
                               "[m] Split a longer move into several "
                               "waypoints, 0 = no split"),
         BT::InputPort<bool>("right_angles", true,
                             "Along the target's X first, then along its "
                             "Y, never diagonally"),
         BT::InputPort<double>("settle_s", 10.0,
                               "[s] Close to the target and no closer for "
                               "this long: done"),
         BT::InputPort<double>("stuck_s", 30.0,
                               "[s] Not getting closer for this long: "
                               "fail, 0 = never")});
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

std::optional<GoToFrame::Goal> GoToFrame::build_goal(const Pose& target) const {
    const auto mode_name = getInput<std::string>("mode");
    const auto mode = waypoint_mode_from_string(
        mode_name ? *mode_name : std::string("FULL_POSE"));
    if (!mode) {
        spdlog::error("[{}] unknown mode: {}", name(),
                      mode_name ? *mode_name : "");
        return std::nullopt;
    }

    const double max_step = getInput<double>("max_step_m").value_or(0.0);
    const auto vehicle = map_->vehicle_in_odom();

    // Along the target's X first, then along its Y, never diagonally. On a
    // task X points out of it: out to the right distance, then sideways into
    // line. The same in every run, so a route can be checked on paper.
    std::vector<Pose> legs{target};
    const bool right_angles = getInput<bool>("right_angles").value_or(true);
    tf2::Quaternion q;
    tf2::fromMsg(target.orientation, q);
    const double yaw = tf2::getYaw(q);
    if (vehicle && right_angles) {
        const double dx = target.position.x - vehicle->position.x;
        const double dy = target.position.y - vehicle->position.y;
        const double forward = dx * std::cos(yaw) + dy * std::sin(yaw);
        const double sideways = -dx * std::sin(yaw) + dy * std::cos(yaw);
        if (std::abs(forward) > kMinLegM && std::abs(sideways) > kMinLegM) {
            Pose corner = *vehicle;
            corner.position.x += forward * std::cos(yaw);
            corner.position.y += forward * std::sin(yaw);
            corner.position.z = target.position.z;
            legs.insert(legs.begin(), corner);
        }
    }

    // A long leg is driven facing the way it goes, so the camera sees ahead:
    // turn on the spot, drive, and turn to the target's heading at the end.
    // A short one keeps the heading.
    using WM = vortex_msgs::msg::WaypointMode;
    auto with_yaw = [](Pose p, double yaw) {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        p.orientation = tf2::toMsg(q);
        return p;
    };
    Goal goal;
    auto add = [&](const Pose& p, std::uint8_t m) {
        auto wp = make_waypoint(p, m);
        wp.position_tolerance = kPassPositionM;
        wp.orientation_tolerance = kPassOrientationRad;
        goal.waypoints.push_back(wp);
    };
    Pose from = vehicle ? *vehicle : target;
    bool faced = false;
    for (std::size_t l = 0; l < legs.size(); ++l) {
        const Pose& end = legs[l];
        const double dx = end.position.x - from.position.x;
        const double dy = end.position.y - from.position.y;
        double heading = std::atan2(dy, dx);
        if (right_angles) {
            // Along one of the target's axes: face that axis, not the line
            // from where the vehicle happens to stop, so a gate is passed
            // square and a resend does not turn the vehicle.
            const double along = dx * std::cos(yaw) + dy * std::sin(yaw);
            const double across = -dx * std::sin(yaw) + dy * std::cos(yaw);
            if (std::abs(across) <= kMinLegM) {
                heading = along >= 0.0 ? yaw : yaw + M_PI;
            } else if (std::abs(along) <= kMinLegM) {
                heading = across >= 0.0 ? yaw + M_PI_2 : yaw - M_PI_2;
            }
        }
        faced = vehicle && std::hypot(dx, dy) > kFaceLegM;
        if (faced) {
            add(with_yaw(from, heading), WM::FULL_POSE);
        }
        int steps = 1;
        if (max_step > 0.0 && vehicle) {
            steps = std::max(
                1, static_cast<int>(std::ceil(
                       distance(from.position, end.position) / max_step)));
        }
        for (int i = 1; i <= steps; ++i) {
            Pose pose = end;
            const double t = static_cast<double>(i) / steps;
            pose.position.x = from.position.x + t * dx;
            pose.position.y = from.position.y + t * dy;
            pose.position.z =
                from.position.z + t * (end.position.z - from.position.z);
            const bool last = l + 1 == legs.size() && i == steps;
            const bool corner = l + 1 < legs.size() && i == steps;
            if (faced) {
                add(with_yaw(pose, heading), WM::FULL_POSE);
            } else {
                add(pose, last ? mode->mode
                          : corner
                              ? static_cast<std::uint8_t>(WM::FULL_POSE)
                              : static_cast<std::uint8_t>(WM::ONLY_POSITION));
            }
        }
        from = faced ? with_yaw(end, heading) : end;
    }
    if (faced) {
        add(target, mode->mode);
    }
    goal.waypoints.back().position_tolerance = 0.0;  // the default
    goal.waypoints.back().orientation_tolerance = 0.0;
    return goal;
}

std::optional<GoToFrame::Goal> GoToFrame::make_goal() {
    frozen_ = false;
    lost_ = false;
    progress_t_ = 0.0;

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

std::optional<BT::NodeStatus> GoToFrame::watch() {
    const auto vehicle = map_->vehicle_in_odom();
    if (!vehicle) {
        return std::nullopt;
    }
    const double t = node_->now().seconds();
    const double m = distance(vehicle->position, last_target_.position);
    tf2::Quaternion qv, qt;
    tf2::fromMsg(vehicle->orientation, qv);
    tf2::fromMsg(last_target_.orientation, qt);
    const auto mode = getInput<std::string>("mode").value_or("FULL_POSE");
    const double deg =
        mode == "ONLY_POSITION"
            ? 0.0
            : std::abs(std::remainder(tf2::getYaw(qt) - tf2::getYaw(qv),
                                      2.0 * M_PI)) *
                  kRad2Deg;

    // A new target starts the clock again.
    if (distance(watched_target_.position, last_target_.position) > 1e-3 ||
        rotation_deg(watched_target_.orientation, last_target_.orientation) >
            0.1 ||
        progress_t_ == 0.0) {
        watched_target_ = last_target_;
        best_m_ = m;
        best_deg_ = deg;
        progress_t_ = t;
        return std::nullopt;
    }
    // Closer by a few cm or degrees counts, the last bit is slow.
    if (m < best_m_ - kProgressM || deg < best_deg_ - kProgressDeg) {
        best_m_ = std::min(best_m_, m);
        best_deg_ = std::min(best_deg_, deg);
        progress_t_ = t;
        return std::nullopt;
    }
    const double idle = t - progress_t_;
    const double settle = getInput<double>("settle_s").value_or(10.0) +
                          getInput<double>("hold_s").value_or(0.0);
    if (m < kSettleM && deg < kSettleDeg && idle > settle) {
        spdlog::info(
            "[{}] {:.2f} m, {:.0f} deg from the target and no closer "
            "for {:.0f} s: there",
            name(), m, deg, idle);
        return BT::NodeStatus::SUCCESS;
    }
    const double stuck = getInput<double>("stuck_s").value_or(30.0);
    if (stuck > 0.0 && idle > stuck) {
        spdlog::warn(
            "[{}] {:.2f} m, {:.0f} deg from the target and no closer "
            "for {:.0f} s: stuck",
            name(), m, deg, idle);
        return BT::NodeStatus::FAILURE;
    }
    return std::nullopt;
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
    // NOTE checks total rotation, not per level
    const double resend_deg = getInput<double>("resend_deg").value_or(10);

    if (vehicle &&
        distance(vehicle->position, last_target_.position) < dead_reckoning &&
        rotation_deg(vehicle->orientation, last_target_.orientation) <
            resend_deg) {
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

    if (distance(last_target_.position, target->position) < resend_m &&
        rotation_deg(last_target_.orientation, target->orientation) <
            resend_deg) {
        return std::nullopt;
    }
    last_target_ = *target;
    return build_goal(*target);
}

}  // namespace vortex_bt_nodes::map
