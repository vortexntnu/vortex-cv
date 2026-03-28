#include "bearing_localization/ros/debug_markers.hpp"

#include <visualization_msgs/msg/marker.hpp>

namespace bearing_localization {

visualization_msgs::msg::MarkerArray build_debug_markers(
    const LocalizationResult& result,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(del);

    int id = 0;
    for (const auto& ray : result.inlier_rays) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = stamp;
        arrow.header.frame_id = frame_id;
        arrow.ns = "bearing_rays";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.10;
        arrow.scale.z = 0.10;
        arrow.color.r = 0.2f;
        arrow.color.g = 0.6f;
        arrow.color.b = 1.0f;
        arrow.color.a = 0.7f;
        arrow.lifetime = rclcpp::Duration::from_seconds(1.0);

        constexpr double ray_display_len = 3.0;
        geometry_msgs::msg::Point start, end;
        start.x = ray.origin_world.x();
        start.y = ray.origin_world.y();
        start.z = ray.origin_world.z();
        end.x =
            ray.origin_world.x() + ray.direction_world.x() * ray_display_len;
        end.y =
            ray.origin_world.y() + ray.direction_world.y() * ray_display_len;
        end.z =
            ray.origin_world.z() + ray.direction_world.z() * ray_display_len;
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        array.markers.push_back(arrow);
    }

    visualization_msgs::msg::Marker sphere;
    sphere.header.stamp = stamp;
    sphere.header.frame_id = frame_id;
    sphere.ns = "bearing_target";
    sphere.id = 0;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = result.position.x();
    sphere.pose.position.y = result.position.y();
    sphere.pose.position.z = result.position.z();
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.3;
    sphere.scale.y = 0.3;
    sphere.scale.z = 0.3;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.35f;
    sphere.color.b = 0.0f;
    sphere.color.a = 1.0f;
    sphere.lifetime = rclcpp::Duration::from_seconds(1.0);
    array.markers.push_back(sphere);

    return array;
}

}  // namespace bearing_localization
