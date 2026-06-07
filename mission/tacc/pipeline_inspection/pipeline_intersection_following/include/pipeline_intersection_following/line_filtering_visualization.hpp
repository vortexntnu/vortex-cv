#pragma once
#include <rclcpp/rclcpp.hpp>
#include <pipeline_intersection_following/line_filtering_ros.hpp>

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vortex_filtering/vortex_filtering.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

visualization_msgs::msg::MarkerArray visualize_track_gates(
    const std::vector<Track>& tracks,
    const rclcpp::Time & timestamp,
    const std::string & frame_id,
    double gate_threshold,
    double gate_min_threshold,
    double gate_max_threshold,
    bool red,
    double orca_depth,
    double dvl_altitude);

visualization_msgs::msg::MarkerArray visualize_line_tracks(const std::vector<Track>& tracks,
    const rclcpp::Time & timestamp,
    const std::string & frame_id,
    double orca_depth,
    double dvl_altitude);