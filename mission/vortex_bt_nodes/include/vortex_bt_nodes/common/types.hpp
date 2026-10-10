#ifndef VORTEX_BT_NODES__COMMON__TYPES_HPP_
#define VORTEX_BT_NODES__COMMON__TYPES_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <string>
#include <string_view>
#include <vector>
#include <vortex_msgs/msg/landmark_track_array.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

/**
 * Shared blackboard types. Poses are in odom (x forward, y right, z down)
 * unless a port says otherwise.
 *
 * In XML a Pose is "x;y;z" or "x;y;z;yaw_deg", a PoseList is poses separated
 * by '|' and an IdList is "3;7;12". Landmark types, subtypes and waypoint
 * modes are written by their message constant name, e.g. type="SLALOM_PIPE"
 * subtype="SLALOM_PIPE_RED". Subtype "ANY" matches every subtype.
 */
namespace vortex_bt_nodes {

using Pose = geometry_msgs::msg::Pose;
using PoseList = std::vector<Pose>;
using IdList = std::vector<int>;
using LandmarkTrack = vortex_msgs::msg::LandmarkTrack;
using LandmarkMap = vortex_msgs::msg::LandmarkTrackArray;

struct MissionClock {
    rclcpp::Time start;
    double run_time_s{900.0};

    double elapsed_s(const rclcpp::Time& now) const {
        return (now - start).seconds();
    }
    double remaining_s(const rclcpp::Time& now) const {
        return run_time_s - elapsed_s(now);
    }
};

inline double yaw_of(const Pose& pose) {
    const auto& q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/** @brief Level pose, yaw in rad. */
inline Pose make_pose(double x, double y, double z, double yaw = 0.0) {
    Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
}

std::optional<std::uint16_t> landmark_type_from_string(std::string_view name);

/** @brief Returns -1 for "ANY". */
std::optional<int> landmark_subtype_from_string(std::string_view name);

std::optional<vortex_msgs::msg::WaypointMode> waypoint_mode_from_string(
    const std::string& name);

/** @brief subtype -1 matches any. */
inline bool matches(const LandmarkTrack& track,
                    std::uint16_t type,
                    int subtype) {
    return track.landmark.type.value == type &&
           (subtype < 0 || track.landmark.subtype.value == subtype);
}

}  // namespace vortex_bt_nodes

namespace BT {

template <>
inline vortex_bt_nodes::Pose convertFromString(StringView str) {
    std::vector<StringView> parts;
    for (auto part : splitString(str, ';')) {
        while (!part.empty() && part.front() == ' ') {
            part.remove_prefix(1);
        }
        while (!part.empty() && part.back() == ' ') {
            part.remove_suffix(1);
        }
        parts.push_back(part);
    }
    if (parts.size() != 3 && parts.size() != 4) {
        throw RuntimeError("Pose must be \"x;y;z\" or \"x;y;z;yaw_deg\", got: ",
                           str);
    }
    const double yaw_deg =
        parts.size() == 4 ? convertFromString<double>(parts[3]) : 0.0;
    return vortex_bt_nodes::make_pose(convertFromString<double>(parts[0]),
                                      convertFromString<double>(parts[1]),
                                      convertFromString<double>(parts[2]),
                                      yaw_deg * M_PI / 180.0);
}

template <>
inline vortex_bt_nodes::PoseList convertFromString(StringView str) {
    vortex_bt_nodes::PoseList poses;
    for (const auto& part : splitString(str, '|')) {
        poses.push_back(convertFromString<vortex_bt_nodes::Pose>(part));
    }
    return poses;
}

}  // namespace BT

#endif  // VORTEX_BT_NODES__COMMON__TYPES_HPP_
