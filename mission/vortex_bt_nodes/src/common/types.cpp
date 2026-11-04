#include "vortex_bt_nodes/common/types.hpp"

#include <stdexcept>
#include <vortex/utils/ros/landmark_names.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>

namespace vortex_bt_nodes {

std::optional<std::uint16_t> landmark_type_from_string(std::string_view name) {
    return vortex::utils::landmarks::landmark_type_from_string(name);
}

std::optional<int> landmark_subtype_from_string(std::string_view name) {
    if (name == "ANY") {
        return -1;
    }
    return vortex::utils::landmarks::landmark_subtype_from_string(name);
}

std::optional<vortex_msgs::msg::WaypointMode> waypoint_mode_from_string(
    const std::string& name) {
    try {
        return vortex::utils::waypoints::waypoint_mode_to_ros(
            vortex::utils::waypoints::string_to_waypoint_mode(name));
    } catch (const std::runtime_error&) {
        return std::nullopt;
    }
}

}  // namespace vortex_bt_nodes
