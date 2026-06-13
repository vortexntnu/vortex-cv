#pragma once

#include <chrono>
#include <cstdio>
#include <deque>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <pipeline_intersection_following/track_manager.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pipeline_intersection_following/line_filtering_visualization.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/// One candidate corner: two confirmed line tracks that keep crossing at the
/// same world point for kJunctionConfirmHits consecutive ticks.
struct JunctionVote {
    int id1 = -1, id2 = -1;
    int hits = 0;
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    /// Far endpoints of each arm (away from the crossing), used for arm
    /// selection.
    Eigen::Matrix<double, 2, 2> line_points =
        Eigen::Matrix<double, 2, 2>::Zero();
};

struct LineIntersection {
    double x;
    double y;
    int id1;
    int id2;
    Eigen::Matrix<double, 2, 2> line_points;

    // Compares by position only — same track IDs can appear at different corners
    bool operator==(const LineIntersection& other) const {
        return (Eigen::Vector2d(x, y) - Eigen::Vector2d(other.x, other.y))
                   .norm() < 0.5;
    }
};

class LineFilteringNode : public rclcpp::Node {
   public:
    LineFilteringNode();

   private:
    /**
     * @brief Timer callback function.
     */
    void timer_callback();

    // --- Inputs
    // ---------------------------------------------------------------
    rclcpp::Subscription<vortex_msgs::msg::LineSegment2DArray>::SharedPtr
        lines_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_sub_;
    rclcpp::Subscription<vortex_msgs::msg::DVLAltitude>::SharedPtr
        altitude_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // --- Outputs (waypoint manager) ------------------------------------------
    rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr waypoint_client_;

    // --- Mission control (FSM) -----------------------------------------------
    /// Trigger server the FSM toggles to start/stop pipeline following.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    /// Trigger client used to report end-of-pipeline to the FSM.
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finished_client_;

    // --- Debug visualization
    // --------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_1_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_3_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_4_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        scene_update_line_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        scene_update_intersection_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        line_points_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        line_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        line_intersection_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        line_intersection_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        line_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
        termination_track_count_pub_;

    std::string target_frame_;
    std::string camera_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool camera_info_received_ = false;
    cv::Mat K_;

    double altitude_ = -1.0;  ///< DVL altitude (camera-to-ground), metres
    geometry_msgs::msg::Pose orca_pose_;

    bool is_executing_action_ = false;  ///< True while the FSM has us following

    // --- Callbacks
    // ------------------------------------------------------------
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void line_callback(
        const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg);
    void altitude_callback(const vortex_msgs::msg::DVLAltitude::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void start_following_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);

    // --- Tracking / intersection logic ---------------------------------------
    void find_new_line_intersections();
    bool find_intersection(const Eigen::Matrix<double, 2, 2>& line1,
                           const Eigen::Matrix<double, 2, 2>& line2,
                           Eigen::Vector2d& intersection,
                           double min_angle);
    bool new_intersection_available();
    void publish_intersection();
    void check_junction_convergence();
    int get_track_by_id(Track& line_track, int id);
    void set_next_line(const JunctionVote& vote);
    void find_and_publish_initial_waypoint();
    void publish_waypoint();
    /**
     * @brief Emit a following waypoint toward (target_x, target_y).
     *
     * Sends XY_AND_YAW with pipe_yaw derived from the track's endpoint pair so
     * the vehicle faces along the pipe, not toward the endpoint. Lateral
     * re-centering is handled by the DP controller's XY axis.
     *
     * @return true (waypoint sent or skipped due to throttle).
     */
    bool follow_toward(double target_x, double target_y, double pipe_yaw);
    void get_track_by_yaw(Track& line_track);
    void termination_check();

    // --- Waypoint sending
    // -----------------------------------------------------
    struct WaypointRequest {
        vortex_msgs::msg::Waypoint wp;
        bool overwrite_prior;
        bool take_priority;
        double switching_threshold;
    };

    /// Builds a Waypoint in the target (odom) frame.
    vortex_msgs::msg::Waypoint make_waypoint(double x,
                                             double y,
                                             double yaw,
                                             uint8_t mode) const;
    /// Queues a waypoint request and kicks the send pump.
    void enqueue_waypoint(const vortex_msgs::msg::Waypoint& wp,
                          bool overwrite_prior,
                          bool take_priority,
                          double switching_threshold);
    void try_send_next_request();

    std::deque<WaypointRequest> request_queue_;
    bool request_in_flight_ = false;
    rclcpp::TimerBase::SharedPtr request_delay_timer_;

    double target_altitude_ = 0.5;      ///< Desired following altitude
    double switching_threshold_ = 0.3;  ///< Default waypoint switching radius
    double realign_yaw_threshold_ =
        0.175;  ///< Rotate-first if bearing error exceeds this (rad)

    // Resend throttling: avoid re-issuing a waypoint (which wipes the queue and
    // resets the reference filter) unless the target moved/rotated
    // meaningfully.
    double min_wp_dist_ =
        0.2;  ///< Min target move (m) before resending a translate wp
    double min_wp_yaw_ =
        0.1;  ///< Min target yaw change (rad) before resending a rotate wp
    int max_resend_skips_ =
        20;  ///< Force a resend after this many consecutive skips
    bool have_prev_wp_ = false;
    double prev_wp_x_ = 0.0;
    double prev_wp_y_ = 0.0;
    double prev_wp_yaw_ = 0.0;
    uint8_t prev_wp_mode_ = 255;
    int wp_skip_count_ = 0;

    TrackManager line_tracker_;
    std::vector<LineIntersection> used_line_intersections_;

    Eigen::Array<double, 2, Eigen::Dynamic> measurements_;

    /// Vote accumulator for junction detection.
    std::vector<JunctionVote> junction_votes_;
    static constexpr int kJunctionConfirmHits = 3;

    int current_line_id_ = -1;
    int termination_counter_ = 0;
    double next_line_yaw_;
    bool junction_in_progress_ =
        false;  ///< True from junction fire until robot reaches junction WP
    double junction_wp_x_ = 0.0;  ///< Camera-adjusted junction X sent to DP
    double junction_wp_y_ = 0.0;  ///< Camera-adjusted junction Y sent to DP
    int junction_hold_ticks_ =
        0;  ///< Consecutive ticks spent waiting to reach junction WP

    bool debug_visualization_ = true;

    // --- Diagnostic logging
    // ---------------------------------------------------
    std::ofstream debug_log_;  ///< Optional file sink for diagnostics.

    /// Logs a printf-style message to the ROS console and (if open) the file.
    /// All arguments must be POD (use .c_str() for std::string).
    template <typename... Args>
    void dlog(const char* fmt, Args... args) {
        char buf[1024];
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
        std::snprintf(buf, sizeof(buf), fmt, args...);
#pragma GCC diagnostic pop
        RCLCPP_INFO(this->get_logger(), "%s", buf);
        if (debug_log_.is_open()) {
            debug_log_ << buf << '\n';
            debug_log_.flush();
        }
    }
};
