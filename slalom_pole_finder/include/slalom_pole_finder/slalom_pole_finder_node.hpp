#pragma once

#include <cstdint>
#include <unordered_set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace slalom_pole_finder
{

class SlalomPoleFinderNode : public rclcpp::Node
{
public:
  SlalomPoleFinderNode();

private:
  struct TrackedPole
  {
    geometry_msgs::msg::Point position;
    std::string id;
  };

  void detectionCallback(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  bool touchesTopEdge(const vision_msgs::msg::Detection2D & detection) const;
  std::string assignId(
    const geometry_msgs::msg::Point & position,
    std::unordered_set<std::size_t> & matched_tracks);

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double object_height_;
  double cam_tx_;
  double cam_ty_;
  double cam_tz_;
  double cam_qx_;
  double cam_qy_;
  double cam_qz_;
  double cam_qw_;
  double top_margin_px_;
  double match_distance_m_;

  std::vector<TrackedPole> tracked_poles_;
  std::string tracking_frame_;
  std::uint64_t next_id_{1};

  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_pose_pub_;
};

}  // namespace slalom_pole_finder
