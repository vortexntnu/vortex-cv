#ifndef BEARING_LOCALIZATION__ROS__DEBUG_MARKERS_HPP_
#define BEARING_LOCALIZATION__ROS__DEBUG_MARKERS_HPP_

#include "bearing_localization/lib/bearing_localizer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

namespace bearing_localization {

visualization_msgs::msg::MarkerArray build_debug_markers(
    const LocalizationResult& result,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__ROS__DEBUG_MARKERS_HPP_
