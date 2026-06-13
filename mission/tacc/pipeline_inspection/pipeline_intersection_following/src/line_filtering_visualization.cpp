#include <pipeline_intersection_following/line_filtering_visualization.hpp>

visualization_msgs::msg::MarkerArray visualize_track_gates(
    const std::vector<Track>& tracks,
    const rclcpp::Time& timestamp,
    const std::string& frame_id,
    double gate_max_threshold,
    bool red,
    double orca_depth,
    double dvl_altitude) {
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& track : tracks) {
        const Eigen::Vector2d position = track.midpoint();
        const double radius = gate_max_threshold;

        // Create a cylinder marker for the track gate
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = timestamp;
        marker.ns = "track_gates";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(5, 0);
        marker.frame_locked = false;
        marker.pose.position.x = position(0);
        marker.pose.position.y = position(1);
        marker.pose.position.z = orca_depth + dvl_altitude;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = 2.0;
        if (red) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.3;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.3;
        }

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

visualization_msgs::msg::MarkerArray visualize_line_tracks(
    const std::vector<Track>& tracks,
    const rclcpp::Time& timestamp,
    const std::string& frame_id,
    double orca_depth,
    double dvl_altitude) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = "track_points";
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    for (const auto& track : tracks) {
        if (!track.confirmed) {
            continue;
        }

        geometry_msgs::msg::Point start;
        start.x = track.line_points(0, 0);
        start.y = track.line_points(1, 0);
        start.z = orca_depth + dvl_altitude;

        geometry_msgs::msg::Point end;
        end.x = track.line_points(0, 1);
        end.y = track.line_points(1, 1);
        end.z = orca_depth + dvl_altitude;

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    marker_array.markers.push_back(marker);

    return marker_array;
}
