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
 * The types that go between nodes written by different people. A node reads
 * and writes these keys with these types, so it can be written and tested
 * alone by filling the blackboard by hand.
 *
 *   Key                          Type          Written by
 *   {pose}                       Pose          PoseFeeder (vehicle in odom)
 *   {map}                        LandmarkMap   MapFeeder (object_map)
 *   {course_frame}               Pose          CourseFrameFeeder
 *   {mission_clock}              MissionClock  MissionClock
 *   {start_pose}                 Pose          SavePose
 *   {role}                       std::string   SelectGatePanel
 *                                              ("survey_repair" or
 *                                               "search_rescue")
 *   {gate_side}                  std::string   SelectGatePanel ("left",
 *                                              "right")
 *   {gap_pose}                   Pose          MatchPipes
 *   {passed_red_ids}             IdList        RecordLayer
 *   {avoid_path}                 PoseList      AvoidSlalom
 *   {bin_subtype}, ...           std::string   ResolveRole (subtype names)
 *   every key in mission.yaml    double or     LoadMissionConfig
 *                                std::string
 *
 * Landmark ids (ports id, gate_id, red_id) are int, the track id in {map}.
 * Landmark types and subtypes are written by name, as in LandmarkType.msg and
 * LandmarkSubtype.msg: type="SLALOM_PIPE" subtype="SLALOM_PIPE_RED"; subtype
 * "ANY" matches every subtype. Waypoint modes (port mode) are written the
 * same way, as in WaypointMode.msg: mode="POSITION_AND_YAW".
 *
 * Poses are in odom (x forward, y right, z down) unless a port says
 * otherwise. In XML a Pose is "x;y;z" or "x;y;z;yaw_deg", e.g.
 * pose="2.0;0.0;1.5;90"; a PoseList is poses separated by '|'; an IdList
 * is "3;7;12".
 */
namespace vortex_bt_nodes {

using Pose = geometry_msgs::msg::Pose;
using PoseList = std::vector<Pose>;
using IdList = std::vector<int>;
using LandmarkTrack = vortex_msgs::msg::LandmarkTrack;
using LandmarkMap = vortex_msgs::msg::LandmarkTrackArray;

/** @brief When the run started and how long it may last. */
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

/** @brief Yaw [rad] of a pose's orientation. */
inline double yaw_of(const Pose& pose) {
    const auto& q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/** @brief Level pose (roll = pitch = 0) at (x, y, z) with yaw [rad]. */
inline Pose make_pose(double x, double y, double z, double yaw = 0.0) {
    Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
}

/** @brief LandmarkType value for a name ("GATE"), or nullopt. */
std::optional<std::uint16_t> landmark_type_from_string(std::string_view name);

/**
 * @brief LandmarkSubtype value for a name ("SLALOM_PIPE_RED"), -1 for
 * "ANY", or nullopt for an unknown name.
 */
std::optional<int> landmark_subtype_from_string(std::string_view name);

/**
 * @brief WaypointMode for a name ("POSITION_AND_YAW" or "position_and_yaw"),
 * or nullopt. Wraps vortex_utils' string_to_waypoint_mode, without the
 * exception, so a bad port fails the node instead of the tree.
 */
std::optional<vortex_msgs::msg::WaypointMode> waypoint_mode_from_string(
    const std::string& name);

/** @brief True if the track has this type and subtype (-1 = any subtype). */
inline bool matches(const LandmarkTrack& track,
                    std::uint16_t type,
                    int subtype) {
    return track.landmark.type.value == type &&
           (subtype < 0 || track.landmark.subtype.value == subtype);
}

}  // namespace vortex_bt_nodes

namespace BT {

/** @brief "x;y;z" or "x;y;z;yaw_deg" → level Pose. Spaces are ignored. */
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

/** @brief Poses separated by '|': "1;0;1 | 2;0;1;90". */
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
