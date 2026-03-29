#ifndef BEARING_LOCALIZATION__ROS__DEBUG_MARKERS_HPP_
#define BEARING_LOCALIZATION__ROS__DEBUG_MARKERS_HPP_

#include "bearing_localization/lib/bearing_localizer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

namespace bearing_localization {

/**
 * @brief Build a MarkerArray visualising a localisation result.
 *
 * Creates arrow markers for each inlier ray and a sphere marker at the
 * estimated target position, suitable for display in Foxglove.
 *
 * @param result   Localisation result containing position and inlier rays.
 * @param frame_id TF frame in which to publish the markers.
 * @param stamp    Timestamp for the marker header.
 * @return MarkerArray ready to publish.
 */
visualization_msgs::msg::MarkerArray build_debug_markers(
    const LocalizationResult& result,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__ROS__DEBUG_MARKERS_HPP_
