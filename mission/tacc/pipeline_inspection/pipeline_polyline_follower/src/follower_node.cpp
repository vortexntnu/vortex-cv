#include "pipeline_polyline_follower/follower_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <limits>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "pipeline_polyline_follower/skeleton.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace {
geometry_msgs::msg::Quaternion quat_from_yaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
}
}  // namespace

PipelinePolylineFollowerNode::PipelinePolylineFollowerNode()
    : Node("pipeline_polyline_follower") {
    // ── Parameters ────────────────────────────────────────────────────────
    odom_frame_ = declare_parameter<std::string>("odom_frame", "nautilus/odom");
    camera_frame_ = declare_parameter<std::string>(
        "camera_frame", "nautilus/downwards_camera_optical");
    target_altitude_ = declare_parameter<double>("target_altitude", 0.8);
    lookahead_dist_ = declare_parameter<double>("lookahead_distance", 1.5);
    dp_epsilon_px_ = declare_parameter<double>("dp_epsilon_px", 3.0);
    min_mask_fill_ = declare_parameter<double>("min_mask_fill", 200.0);
    switching_threshold_ =
        declare_parameter<double>("switching_threshold", 0.3);
    wp_alpha_ = declare_parameter<double>("wp_smoothing_alpha", 0.4);
    max_detection_dist_ = declare_parameter<double>("max_detection_dist", 4.0);
    min_wp_dist_ = declare_parameter<double>("min_wp_dist", 0.1);
    min_wp_yaw_ = declare_parameter<double>("min_wp_yaw", 0.05);
    watchdog_timeout_s_ = declare_parameter<double>("watchdog_timeout_s", 3.0);

    const std::string mask_topic = declare_parameter<std::string>(
        "mask_topic", "/pipeline/down_camera/segmentation_mask");
    const std::string cam_info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/pipeline/down_camera/camera_info");
    const std::string odom_topic =
        declare_parameter<std::string>("odom_topic", "/nautilus/odom");
    const std::string alt_topic = declare_parameter<std::string>(
        "altitude_topic", "/nautilus/dvl/altitude");
    const std::string wp_service = declare_parameter<std::string>(
        "waypoint_service", "/nautilus/waypoint_addition");
    const std::string start_srv_name = declare_parameter<std::string>(
        "start_service",
        "/nautilus/pipeline_inspection_fsm/start_pipeline_following");
    const std::string stop_srv_name = declare_parameter<std::string>(
        "stop_service", "/nautilus/pipeline_inspection_fsm/pipeline_finished");

    // ── TF ────────────────────────────────────────────────────────────────
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_iface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_iface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // ── QoS ───────────────────────────────────────────────────────────────
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto sensor_qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // ── Subscriptions ─────────────────────────────────────────────────────
    mask_sub_ = create_subscription<sensor_msgs::msg::Image>(
        mask_topic, sensor_qos,
        std::bind(&PipelinePolylineFollowerNode::mask_callback, this, _1));

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_info_topic, rclcpp::QoS(1).transient_local(),
        std::bind(&PipelinePolylineFollowerNode::camera_info_callback, this,
                  _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, sensor_qos,
        std::bind(&PipelinePolylineFollowerNode::odom_callback, this, _1));

    alt_sub_ = create_subscription<vortex_msgs::msg::DVLAltitude>(
        alt_topic, sensor_qos,
        std::bind(&PipelinePolylineFollowerNode::altitude_callback, this, _1));

    // ── Services ──────────────────────────────────────────────────────────
    start_srv_ = create_service<std_srvs::srv::Trigger>(
        start_srv_name,
        std::bind(&PipelinePolylineFollowerNode::start_cb, this, _1, _2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        stop_srv_name,
        std::bind(&PipelinePolylineFollowerNode::stop_cb, this, _1, _2));

    // ── Waypoint client ───────────────────────────────────────────────────
    wp_client_ = create_client<vortex_msgs::srv::SendWaypoints>(wp_service);

    // ── Debug publisher ───────────────────────────────────────────────────
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image",
                                                           rclcpp::QoS(1));

    // ── Watchdog timer ────────────────────────────────────────────────────
    // Fires every second; warns if active_ but no valid frame has arrived
    // recently (camera dropout, topic death, etc.)
    watchdog_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        if (!active_) {
            watchdog_warned_ = false;
            return;
        }
        const auto age = (this->now() - last_valid_frame_).seconds();
        if (age > watchdog_timeout_s_) {
            if (!watchdog_warned_) {
                RCLCPP_WARN(get_logger(),
                            "No valid mask frame for %.1f s while active — "
                            "check segmentation pipeline",
                            age);
                watchdog_warned_ = true;
            }
        } else {
            watchdog_warned_ = false;
        }
    });

    RCLCPP_INFO(get_logger(), "pipeline_polyline_follower ready (inactive)");
}

// ── Service callbacks ─────────────────────────────────────────────────────

void PipelinePolylineFollowerNode::start_cb(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& /*req*/,
    const std_srvs::srv::Trigger::Response::SharedPtr& res) {
    polyline_state_ = PolylineState{};
    wp_smooth_init_ = false;
    have_last_sent_ = false;
    last_valid_frame_ = this->now();
    watchdog_warned_ = false;
    active_ = true;
    res->success = true;
    res->message = "pipeline_polyline_follower started";
    RCLCPP_INFO(get_logger(), "Started following");
}

void PipelinePolylineFollowerNode::stop_cb(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& /*req*/,
    const std_srvs::srv::Trigger::Response::SharedPtr& res) {
    active_ = false;
    res->success = true;
    res->message = "pipeline_polyline_follower stopped";
    RCLCPP_INFO(get_logger(), "Stopped following");
}

// ── Sensor callbacks ──────────────────────────────────────────────────────

void PipelinePolylineFollowerNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    camera_info_received_ = true;
    cam_info_sub_.reset();  // one-shot: unsubscribe after first message
}

void PipelinePolylineFollowerNode::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    vehicle_pos_xy_.x() = msg->pose.pose.position.x;
    vehicle_pos_xy_.y() = msg->pose.pose.position.y;
    vehicle_z_ = msg->pose.pose.position.z;
    vehicle_vel_xy_.x() = msg->twist.twist.linear.x;
    vehicle_vel_xy_.y() = msg->twist.twist.linear.y;
}

void PipelinePolylineFollowerNode::altitude_callback(
    const vortex_msgs::msg::DVLAltitude::ConstSharedPtr& msg) {
    altitude_ = msg->altitude;
}

// ── Main processing ───────────────────────────────────────────────────────

void PipelinePolylineFollowerNode::mask_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!active_)
        return;
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for camera info");
        return;
    }
    if (altitude_ < 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for DVL altitude");
        return;
    }

    // ── Convert to mono8 binary ───────────────────────────────────────────
    cv::Mat mono;
    try {
        const auto cv_img = cv_bridge::toCvShare(msg);
        if (cv_img->image.channels() > 1)
            cv::cvtColor(cv_img->image, mono, cv::COLOR_BGR2GRAY);
        else
            mono = cv_img->image.clone();
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        return;
    }
    cv::threshold(mono, mono, 127, 255, cv::THRESH_BINARY);

    const int fill = cv::countNonZero(mono);
    if (fill < static_cast<int>(min_mask_fill_))
        return;  // too sparse

    // ── Skeletonize ───────────────────────────────────────────────────────
    zhang_suen_thin(mono);

    // ── Trace to ordered pixel path ───────────────────────────────────────
    const std::vector<cv::Point> pixel_path = trace_skeleton(mono);
    if (pixel_path.size() < 4)
        return;

    // ── Douglas-Peucker simplification ───────────────────────────────────
    std::vector<cv::Point2f> fp(pixel_path.begin(), pixel_path.end());
    std::vector<cv::Point2f> approx;
    cv::approxPolyDP(fp, approx, dp_epsilon_px_, false);
    if (approx.size() < 2)
        return;

    std::vector<cv::Point> dp_pts(approx.begin(), approx.end());

    // ── Project pixels → odom XY ──────────────────────────────────────────
    std::vector<Eigen::Vector2d> odom_pts;
    if (!project_to_odom(dp_pts, msg->header, odom_pts))
        return;

    // ── Sanity check: reject frames where the pipe is implausibly far ──────
    // A bad mask (turbidity, shadow) can project to a wildly wrong location.
    // We discard the frame if no detected point is within max_detection_dist_
    // of the vehicle.
    {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& pt : odom_pts)
            min_dist = std::min(min_dist, (pt - vehicle_pos_xy_).norm());
        if (min_dist > max_detection_dist_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropped frame: nearest detected point %.2f m from vehicle "
                "(max %.2f m)",
                min_dist, max_detection_dist_);
            return;
        }
    }

    // ── Update directed polyline state ────────────────────────────────────
    update_polyline(polyline_state_, odom_pts, vehicle_pos_xy_,
                    vehicle_vel_xy_);

    // ── Advance monotone arc-length cursor ────────────────────────────────
    advance_cursor(polyline_state_, vehicle_pos_xy_);

    // Frame is valid — reset watchdog
    last_valid_frame_ = this->now();

    // ── Compute raw look-ahead waypoint ───────────────────────────────────
    Eigen::Vector2d wp_xy_raw;
    double yaw_raw{0.0};
    if (!get_lookahead(polyline_state_, lookahead_dist_, wp_xy_raw, yaw_raw))
        return;

    // ── EMA smoothing on look-ahead XY and yaw ────────────────────────────
    // Seed the filter on the first valid output so there's no step from zero.
    if (!wp_smooth_init_) {
        wp_xy_smooth_ = wp_xy_raw;
        wp_yaw_smooth_ = yaw_raw;
        wp_smooth_init_ = true;
    } else {
        wp_xy_smooth_ =
            wp_alpha_ * wp_xy_raw + (1.0 - wp_alpha_) * wp_xy_smooth_;
        // Wrap-safe yaw blend: interpolate on the unit circle.
        const double dy = std::sin(yaw_raw) * wp_alpha_ +
                          std::sin(wp_yaw_smooth_) * (1.0 - wp_alpha_);
        const double dx = std::cos(yaw_raw) * wp_alpha_ +
                          std::cos(wp_yaw_smooth_) * (1.0 - wp_alpha_);
        wp_yaw_smooth_ = std::atan2(dy, dx);
    }

    // ── Rate gate: only send when position or heading changed enough ───────
    if (have_last_sent_) {
        const double d_xy = (wp_xy_smooth_ - last_sent_xy_).norm();
        const double d_yaw =
            std::abs(std::atan2(std::sin(wp_yaw_smooth_ - last_sent_yaw_),
                                std::cos(wp_yaw_smooth_ - last_sent_yaw_)));
        if (d_xy < min_wp_dist_ && d_yaw < min_wp_yaw_) {
            // Still produce debug image with the smoothed point
            if (debug_pub_->get_subscription_count() > 0)
                publish_debug(mono, pixel_path, dp_pts, msg->header,
                              wp_xy_smooth_, wp_yaw_smooth_);
            return;
        }
    }

    last_sent_xy_ = wp_xy_smooth_;
    last_sent_yaw_ = wp_yaw_smooth_;
    have_last_sent_ = true;

    send_waypoint(wp_xy_smooth_, wp_yaw_smooth_);

    // ── Debug visualisation ───────────────────────────────────────────────
    if (debug_pub_->get_subscription_count() > 0)
        publish_debug(mono, pixel_path, dp_pts, msg->header, wp_xy_smooth_,
                      wp_yaw_smooth_);
}

// ── Helpers ───────────────────────────────────────────────────────────────

bool PipelinePolylineFollowerNode::project_to_odom(
    const std::vector<cv::Point>& pixels,
    const std_msgs::msg::Header& header,
    std::vector<Eigen::Vector2d>& odom_pts) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf2_buffer_->lookupTransform(odom_frame_, camera_frame_,
                                                  tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup failed: %s", ex.what());
        return false;
    }

    odom_pts.clear();
    odom_pts.reserve(pixels.size());

    for (const auto& px : pixels) {
        // Pinhole back-projection at depth = altitude
        const double X_cam = (px.x - cx_) * altitude_ / fx_;
        const double Y_cam = (px.y - cy_) * altitude_ / fy_;
        const double Z_cam = altitude_;

        geometry_msgs::msg::PointStamped pt_cam, pt_odom;
        pt_cam.header.frame_id = camera_frame_;
        pt_cam.header.stamp = header.stamp;
        pt_cam.point.x = X_cam;
        pt_cam.point.y = Y_cam;
        pt_cam.point.z = Z_cam;

        tf2::doTransform(pt_cam, pt_odom, tf_stamped);
        odom_pts.emplace_back(pt_odom.point.x, pt_odom.point.y);
    }
    return true;
}

void PipelinePolylineFollowerNode::send_waypoint(const Eigen::Vector2d& xy,
                                                 double yaw) {
    if (!wp_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waypoint service not ready");
        return;
    }

    vortex_msgs::msg::Waypoint wp;
    wp.pose.position.x = xy.x();
    wp.pose.position.y = xy.y();
    wp.pose.position.z =
        vehicle_z_;  // z from odom; altitude controller holds depth
    wp.pose.orientation = quat_from_yaw(yaw);
    wp.waypoint_mode.mode = vortex_msgs::msg::WaypointMode::XY_FORWARD_DIR;
    wp.keep_altitude = true;
    wp.desired_altitude = target_altitude_;
    wp.require_altitude_convergence = false;

    auto req = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
    req->waypoints = {wp};
    req->switching_threshold = switching_threshold_;
    req->overwrite_prior_waypoints =
        true;  // always replace with latest look-ahead
    req->take_priority = false;

    wp_client_->async_send_request(
        req,
        [this](
            rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedFuture f) {
            if (!f.get()->success)
                RCLCPP_WARN(get_logger(), "Waypoint service rejected request");
        });
}

void PipelinePolylineFollowerNode::publish_debug(
    const cv::Mat& mono_mask,
    const std::vector<cv::Point>& skeleton_path,
    const std::vector<cv::Point>& dp_pts,
    const std_msgs::msg::Header& header,
    const Eigen::Vector2d& lookahead_odom,
    double /*lookahead_yaw*/) {
    // BGR canvas: dim mask as background
    cv::Mat canvas;
    cv::cvtColor(mono_mask, canvas, cv::COLOR_GRAY2BGR);
    // Dim it so overlays are easy to see
    canvas *= 0.35;

    // ── Skeleton in green ─────────────────────────────────────────────────
    for (size_t i = 0; i + 1 < skeleton_path.size(); ++i)
        cv::line(canvas, skeleton_path[i], skeleton_path[i + 1],
                 cv::Scalar(0, 220, 0), 1, cv::LINE_AA);

    // ── DP polyline in cyan, vertices as circles ───────────────────────────
    for (size_t i = 0; i + 1 < dp_pts.size(); ++i)
        cv::line(canvas, dp_pts[i], dp_pts[i + 1], cv::Scalar(255, 200, 0), 2,
                 cv::LINE_AA);
    for (const auto& pt : dp_pts)
        cv::circle(canvas, pt, 4, cv::Scalar(255, 200, 0), -1, cv::LINE_AA);

    // Mark entry (first) vertex in white and exit (last) in yellow
    if (!dp_pts.empty()) {
        cv::circle(canvas, dp_pts.front(), 6, cv::Scalar(255, 255, 255), 2,
                   cv::LINE_AA);
        cv::circle(canvas, dp_pts.back(), 6, cv::Scalar(0, 255, 255), 2,
                   cv::LINE_AA);
    }

    // ── Look-ahead point projected back to image ───────────────────────────
    // Re-project the odom look-ahead point back to pixel coords for display.
    // We use the inverse of the pinhole projection (odom→camera→pixel).
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf2_buffer_->lookupTransform(camera_frame_, odom_frame_,
                                                  tf2::TimePointZero);

        geometry_msgs::msg::PointStamped pt_odom, pt_cam;
        pt_odom.header.frame_id = odom_frame_;
        pt_odom.header.stamp = header.stamp;
        pt_odom.point.x = lookahead_odom.x();
        pt_odom.point.y = lookahead_odom.y();
        pt_odom.point.z = vehicle_z_;

        tf2::doTransform(pt_odom, pt_cam, tf_stamped);

        if (pt_cam.point.z > 0.01) {
            const int u =
                static_cast<int>(pt_cam.point.x * fx_ / pt_cam.point.z + cx_);
            const int v =
                static_cast<int>(pt_cam.point.y * fy_ / pt_cam.point.z + cy_);
            if (u >= 0 && u < canvas.cols && v >= 0 && v < canvas.rows) {
                cv::circle(canvas, {u, v}, 10, cv::Scalar(0, 0, 255), -1,
                           cv::LINE_AA);
                cv::circle(canvas, {u, v}, 12, cv::Scalar(255, 255, 255), 2,
                           cv::LINE_AA);
            }
        }
    } catch (const tf2::TransformException&) {
        // Skip reprojection if TF not available — rest of debug image is valid
    }

    // ── Publish ───────────────────────────────────────────────────────────
    auto img_msg = cv_bridge::CvImage(header, "bgr8", canvas).toImageMsg();
    debug_pub_->publish(*img_msg);
}
