#pragma once

#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

#include "pipeline_polyline_follower/polyline.hpp"

class PipelinePolylineFollowerNode : public rclcpp::Node {
   public:
    PipelinePolylineFollowerNode();

   private:
    // ── ROS callbacks ─────────────────────────────────────────────────────
    void mask_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void altitude_callback(
        const vortex_msgs::msg::DVLAltitude::ConstSharedPtr& msg);

    void start_cb(const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
                  const std_srvs::srv::Trigger::Response::SharedPtr& res);
    void stop_cb(const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
                 const std_srvs::srv::Trigger::Response::SharedPtr& res);

    // ── Helpers ───────────────────────────────────────────────────────────

    /**
     * Project an ordered pixel path into 3-D odom-frame XY coordinates using
     * the pinhole model and current DVL altitude.
     * Returns false if camera intrinsics or TF are not yet available.
     */
    bool project_to_odom(const std::vector<cv::Point>& pixels,
                         const std_msgs::msg::Header& header,
                         std::vector<Eigen::Vector2d>& odom_pts);

    /** Send a single waypoint to the waypoint_addition service. */
    void send_waypoint(const Eigen::Vector2d& xy, double yaw);

    /**
     * Publish a BGR debug image: mask background, skeleton in green,
     * DP polyline in blue, look-ahead point in red.
     */
    void publish_debug(const cv::Mat& mono_mask,
                       const std::vector<cv::Point>& skeleton_path,
                       const std::vector<cv::Point>& dp_pts,
                       const std_msgs::msg::Header& header,
                       const Eigen::Vector2d& lookahead_odom,
                       double lookahead_yaw);

    // ── Configuration ─────────────────────────────────────────────────────
    std::string odom_frame_;
    std::string camera_frame_;
    double target_altitude_;
    double lookahead_dist_;
    double dp_epsilon_px_;  ///< Douglas-Peucker tolerance in pixels
    double min_mask_fill_;  ///< skip frames with fewer foreground pixels
    double switching_threshold_;
    // smoothing
    double wp_alpha_;  ///< EMA alpha for look-ahead xy+yaw (0=frozen, 1=raw)
    double max_detection_dist_;  ///< reject frame if nearest detected point is
                                 ///< farther (m)
    // rate limiting
    double min_wp_dist_;  ///< minimum xy change before re-sending (m)
    double min_wp_yaw_;   ///< minimum yaw change before re-sending (rad)
    // watchdog
    double watchdog_timeout_s_;  ///< warn if no valid frame received for this
                                 ///< many seconds

    // ── State ─────────────────────────────────────────────────────────────
    bool active_{false};
    PolylineState polyline_state_;

    double altitude_{-1.0};
    bool camera_info_received_{false};
    double fx_{0}, fy_{0}, cx_{0}, cy_{0};

    Eigen::Vector2d vehicle_pos_xy_{0.0, 0.0};
    Eigen::Vector2d vehicle_vel_xy_{0.0, 0.0};
    double vehicle_z_{0.0};

    // EMA state for look-ahead smoothing
    Eigen::Vector2d wp_xy_smooth_{0.0, 0.0};
    double wp_yaw_smooth_{0.0};
    bool wp_smooth_init_{false};

    // Rate-limiting state
    Eigen::Vector2d last_sent_xy_{0.0, 0.0};
    double last_sent_yaw_{0.0};
    bool have_last_sent_{false};

    // Watchdog state
    rclcpp::Time last_valid_frame_{0, 0, RCL_ROS_TIME};
    bool watchdog_warned_{false};

    // ── ROS handles ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<vortex_msgs::msg::DVLAltitude>::SharedPtr alt_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

    rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr wp_client_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};
