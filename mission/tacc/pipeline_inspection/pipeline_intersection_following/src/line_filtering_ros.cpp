#include <pipeline_intersection_following/line_filtering_ros.hpp>

#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace {
geometry_msgs::msg::Quaternion quat_from_yaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
}
}  // namespace

LineFilteringNode::LineFilteringNode() : Node("line_filtering_node") {
    declare_parameter<double>("clutter_rate", 0.001);
    declare_parameter<double>("probability_of_detection", 0.7);
    declare_parameter<double>("probability_of_survival", 0.99);
    declare_parameter<double>("gate_threshold", 2.5);
    declare_parameter<double>("min_gate_threshold", 1.0);
    declare_parameter<double>("max_gate_threshold", 10.0);
    declare_parameter<double>("confirmation_threshold", 0.9);
    declare_parameter<double>("initial_existence_probability", 0.4);
    declare_parameter<int>("update_interval_ms", 500);
    declare_parameter<double>("std_dynmod", 0.2);
    declare_parameter<double>("std_sensor", 0.5);
    declare_parameter<double>("connected_lines_threshold", 0.5);
    declare_parameter<double>("crossing_min_angle", 0.5236);
    declare_parameter<int>("termination_counter_threshold", 30);

    // N/M track confirmation / deletion windows.
    declare_parameter<int>("nm.confirm_n", 2);
    declare_parameter<int>("nm.confirm_m", 4);
    declare_parameter<int>("nm.delete_n", 4);
    declare_parameter<int>("nm.delete_m", 4);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    target_frame_ =
        this->declare_parameter<std::string>("target_frame", "nautilus/odom");
    auto lines_sub_topic = this->declare_parameter<std::string>(
        "lines_sub_topic", "/irls_line/lines");
    auto camera_info_sub_topic = this->declare_parameter<std::string>(
        "camera_info_sub_topic", "/pipeline/down_camera/camera_info");
    auto altitude_sub_topic = this->declare_parameter<std::string>(
        "altitude_sub_topic", "/nautilus/dvl/altitude");
    auto odom_sub_topic =
        this->declare_parameter<std::string>("odom_sub_topic", "/nautilus/odom");

    auto waypoint_service = this->declare_parameter<std::string>(
        "waypoint_service", "/nautilus/waypoint_addition");
    auto start_service_name = this->declare_parameter<std::string>(
        "start_service",
        "/nautilus/pipeline_inspection_fsm/start_pipeline_following");
    auto finished_service_name = this->declare_parameter<std::string>(
        "finished_service",
        "/nautilus/pipeline_inspection_fsm/pipeline_finished");

    target_altitude_ = this->declare_parameter<double>("target_altitude", 0.5);
    switching_threshold_ =
        this->declare_parameter<double>("switching_threshold", 0.3);
    realign_yaw_threshold_ =
        this->declare_parameter<double>("realign_yaw_threshold", 0.35);
    min_wp_dist_ = this->declare_parameter<double>("min_wp_dist", 0.2);
    min_wp_yaw_ = this->declare_parameter<double>("min_wp_yaw", 0.1);
    max_resend_skips_ = this->declare_parameter<int>("max_resend_skips", 20);

    debug_visualization_ = this->declare_parameter("debug_visualization", true);

    auto debug_log_file = this->declare_parameter<std::string>(
        "debug_log_file", "/tmp/pif_debug.log");
    if (!debug_log_file.empty()) {
        debug_log_.open(debug_log_file, std::ios::out | std::ios::trunc);
        if (debug_log_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Writing diagnostics to %s",
                        debug_log_file.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not open debug log file %s",
                        debug_log_file.c_str());
        }
    }
    dlog("=== pipeline_intersection_following node started ===");

    // --- TF -------------------------------------------------------------------
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // --- Inputs ---------------------------------------------------------------
    lines_sub_ =
        this->create_subscription<vortex_msgs::msg::LineSegment2DArray>(
            lines_sub_topic, qos_sensor_data,
            std::bind(&LineFilteringNode::line_callback, this, _1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::camera_info_callback, this, _1));

    altitude_sub_ = this->create_subscription<vortex_msgs::msg::DVLAltitude>(
        altitude_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::altitude_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::odom_callback, this, _1));

    // --- Outputs / mission control -------------------------------------------
    waypoint_client_ =
        this->create_client<vortex_msgs::srv::SendWaypoints>(waypoint_service);

    finished_client_ =
        this->create_client<std_srvs::srv::Trigger>(finished_service_name);

    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        start_service_name,
        std::bind(&LineFilteringNode::start_following_callback, this, _1, _2));

    // --- Debug visualization --------------------------------------------------
    line_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/line/point", qos_sensor_data);
    line_intersection_pub_ =
        this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/line/intersection", qos_sensor_data);

    if (debug_visualization_) {
        point_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_1", qos_sensor_data);
        point_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_2", qos_sensor_data);
        point_3_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_3", qos_sensor_data);
        point_4_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_4", qos_sensor_data);
        line_points_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/tracks/points", qos_sensor_data);
        scene_update_line_pub_ =
            this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
                "/scene_update", qos_sensor_data);
        scene_update_intersection_pub_ =
            this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
                "/scene_update_intersection", qos_sensor_data);
        line_intersection_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/line/intersection_pose", qos_sensor_data);
        line_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/pose", qos_sensor_data);
    }

    // --- Track managers -------------------------------------------------------
    double std_dynmod = get_parameter("std_dynmod").as_double();
    double std_sensor = get_parameter("std_sensor").as_double();

    NMConfig nm_config;
    nm_config.confirm_n = get_parameter("nm.confirm_n").as_int();
    nm_config.confirm_m = get_parameter("nm.confirm_m").as_int();
    nm_config.delete_n = get_parameter("nm.delete_n").as_int();
    nm_config.delete_m = get_parameter("nm.delete_m").as_int();

    // Orientation association gate (line tracker only) -- keeps a perpendicular
    // line at a junction from being absorbed into the current line track.
    double orientation_gate = this->declare_parameter<double>(
        "orientation_gate_threshold", 0.5236);  // 30 deg

    line_tracker_ = TrackManager();
    line_tracker_.set_dyn_model(std_dynmod);
    line_tracker_.set_sensor_model(std_sensor);
    line_tracker_.set_nm_config(nm_config);
    line_tracker_.set_orientation_gate(orientation_gate);

    line_intersection_tracker_ = TrackManager();
    line_intersection_tracker_.set_dyn_model(std_dynmod);
    line_intersection_tracker_.set_sensor_model(std_sensor);
    line_intersection_tracker_.set_nm_config(nm_config);

    // --- Timer ----------------------------------------------------------------
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_interval),
        std::bind(&LineFilteringNode::timer_callback, this));
}

void LineFilteringNode::start_following_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    is_executing_action_ = !is_executing_action_;
    dlog("START_SERVICE toggled -> following=%d", is_executing_action_ ? 1 : 0);
    if (is_executing_action_) {
        RCLCPP_INFO(this->get_logger(), "Pipeline following started");
        // Reset per-run state so a new run does not reuse stale junctions.
        used_line_intersections_.clear();
        termination_counter_ = 0;
        current_line_id_counter_ = 0;
        have_prev_wp_ = false;
        wp_skip_count_ = 0;
        prev_wp_mode_ = 255;
    } else {
        RCLCPP_INFO(this->get_logger(), "Pipeline following stopped");
    }
    response->success = is_executing_action_;
    response->message = is_executing_action_ ? "following" : "stopped";
}

void LineFilteringNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
    camera_info_received_ = true;
    camera_info_sub_.reset();
}

void LineFilteringNode::altitude_callback(
    const vortex_msgs::msg::DVLAltitude::SharedPtr msg) {
    altitude_ = msg->altitude;
}

void LineFilteringNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    orca_pose_ = msg->pose.pose;
}

void LineFilteringNode::line_callback(
    const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg) {
    dlog("LINE_CB: %zu lines, alt=%.2f, cam_info=%d, frame=%s",
         msg->lines.size(), altitude_, camera_info_received_ ? 1 : 0,
         msg->header.frame_id.c_str());
    if (altitude_ < 0.0) {
        dlog("LINE_CB: dropped -- altitude not yet received");
        return;
    }
    if (!camera_info_received_) {
        dlog("LINE_CB: dropped -- camera info not yet received");
        return;
    }

    const double depth = altitude_;

    // Flatten the line endpoints into a single list of pixel points, two per
    // line segment (p0, p1), matching the measurements_/line_params_ layout.
    std::vector<std::pair<double, double>> pixels;
    pixels.reserve(msg->lines.size() * 2);
    for (const auto& line : msg->lines) {
        pixels.emplace_back(line.p0.x, line.p0.y);
        pixels.emplace_back(line.p1.x, line.p1.y);
    }

    if (pixels.empty()) {
        return;
    }

    try {
        geometry_msgs::msg::TransformStamped transform =
            tf2_buffer_->lookupTransform(target_frame_, msg->header.frame_id,
                                         tf2::TimePointZero);

        const size_t size = pixels.size();
        measurements_ = Eigen::Array<double, 2, Eigen::Dynamic>(2, size);
        line_params_ = Eigen::Array<double, 2, Eigen::Dynamic>(2, size / 2);

        size_t i = 0;
        for (const auto& [u, v] : pixels) {
            // Back-project pixel (u, v) to camera-frame 3D using intrinsics.
            double X = (u - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
            double Y = (v - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);

            tf2::Vector3 point(X, Y, depth);

            tf2::Vector3 transformed =
                tf2::Transform(
                    tf2::Quaternion(transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z,
                                    transform.transform.rotation.w),
                    tf2::Vector3(transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform.transform.translation.z)) *
                point;

            measurements_.col(i) << transformed.x(), transformed.y();
            if (i % 2 == 1) {
                line_params_.col((i - 1) / 2) =
                    (measurements_.col(i - 1) + measurements_.col(i)) / 2.0;
            }

            if (debug_visualization_) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = msg->header.stamp;
                pose_msg.header.frame_id = target_frame_;
                pose_msg.pose.position.x = transformed.x();
                pose_msg.pose.position.y = transformed.y();
                pose_msg.pose.position.z = transformed.z();

                switch (i) {
                    case 0: point_1_->publish(pose_msg); break;
                    case 1: point_2_->publish(pose_msg); break;
                    case 2: point_3_->publish(pose_msg); break;
                    case 3: point_4_->publish(pose_msg); break;
                    default: break;
                }
            }

            i++;
        }
        dlog("LINE_CB: transformed %zu endpoints (%zu line midpoints) to %s",
             size, size / 2, target_frame_.c_str());
        // Log each line's world-frame midpoint and orientation, plus pairwise
        // midpoint separation -- to see whether two detections are distinct
        // lines and how far apart their midpoints are (vs the gate thresholds).
        const Eigen::Index ncols = line_params_.cols();
        for (Eigen::Index c = 0; c < ncols; ++c) {
            const Eigen::Index e0 = 2 * c, e1 = 2 * c + 1;
            const double ang = std::atan2(
                measurements_(1, e1) - measurements_(1, e0),
                measurements_(0, e1) - measurements_(0, e0));
            dlog("  MEAS line %ld: mid=(%.2f,%.2f) ang=%.1fdeg "
                 "p0=(%.2f,%.2f) p1=(%.2f,%.2f)",
                 (long)c, line_params_(0, c), line_params_(1, c),
                 ang * 180.0 / M_PI, measurements_(0, e0), measurements_(1, e0),
                 measurements_(0, e1), measurements_(1, e1));
        }
        if (ncols == 2) {
            const double sep = std::hypot(line_params_(0, 0) - line_params_(0, 1),
                                          line_params_(1, 0) - line_params_(1, 1));
            dlog("  MEAS midpoint separation = %.2f m (min_gate=%.2f, max_gate=%.2f)",
                 sep, get_parameter("min_gate_threshold").as_double(),
                 get_parameter("max_gate_threshold").as_double());
        }

    } catch (tf2::TransformException& ex) {
        dlog("LINE_CB: TRANSFORM FAILED %s -> %s : %s",
             msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    }
}

// --- Waypoint sending --------------------------------------------------------

vortex_msgs::msg::Waypoint LineFilteringNode::make_waypoint(
    double x, double y, double yaw, uint8_t mode) const {
    vortex_msgs::msg::Waypoint wp;
    wp.pose.position.x = x;
    wp.pose.position.y = y;
    wp.pose.position.z = orca_pose_.position.z;
    wp.pose.orientation = quat_from_yaw(yaw);
    wp.waypoint_mode.mode = mode;
    // Hold altitude for translating waypoints, but not for orientation-only ones
    // -- commanding altitude there can destabilise heading convergence.
    wp.keep_altitude = (mode != vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION);
    wp.desired_altitude = target_altitude_;
    return wp;
}

void LineFilteringNode::enqueue_waypoint(const vortex_msgs::msg::Waypoint& wp,
                                         bool overwrite_prior,
                                         bool take_priority,
                                         double switching_threshold) {
    request_queue_.push_back({wp, overwrite_prior, take_priority,
                              switching_threshold});
    try_send_next_request();
}

void LineFilteringNode::try_send_next_request() {
    if (request_in_flight_ || request_queue_.empty()) {
        return;
    }
    if (!waypoint_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waypoint service not ready.");
        return;
    }

    request_in_flight_ = true;
    auto item = request_queue_.front();
    request_queue_.pop_front();

    auto req = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
    req->waypoints = {item.wp};
    req->overwrite_prior_waypoints = item.overwrite_prior;
    req->take_priority = item.take_priority;
    req->switching_threshold = item.switching_threshold;

    waypoint_client_->async_send_request(
        req,
        [this](rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedFuture
                   future) {
            request_in_flight_ = false;
            try {
                auto resp = future.get();
                RCLCPP_INFO(this->get_logger(), "Waypoint sent. success=%d",
                            resp->success);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Waypoint send failed.");
            }
            request_delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(200), [this]() {
                    request_delay_timer_->cancel();
                    try_send_next_request();
                });
        });
}

void LineFilteringNode::timer_callback() {
    if (!is_executing_action_) {
        return;
    }

    int update_interval = get_parameter("update_interval_ms").as_int();
    double confirmation_threshold =
        get_parameter("confirmation_threshold").as_double();
    double gate_threshold = get_parameter("gate_threshold").as_double();
    double min_gate_threshold = get_parameter("min_gate_threshold").as_double();
    double max_gate_threshold = get_parameter("max_gate_threshold").as_double();
    double prob_of_detection =
        get_parameter("probability_of_detection").as_double();
    double prob_of_survival =
        get_parameter("probability_of_survival").as_double();
    double clutter_intensity = get_parameter("clutter_rate").as_double();
    double initial_existence_probability =
        get_parameter("initial_existence_probability").as_double();

    // Update line tracks
    line_tracker_.update_line_tracks(
        measurements_, line_params_, update_interval, confirmation_threshold,
        gate_threshold, min_gate_threshold, max_gate_threshold,
        prob_of_detection, prob_of_survival, clutter_intensity,
        initial_existence_probability);

    measurements_.resize(2, 0);
    line_params_.resize(2, 0);

    // delete tracks
    line_tracker_.delete_tracks();

    // find line crossings
    find_new_line_intersections();

    line_intersection_tracker_.update_line_intersection_tracks(
        current_line_intersections_, current_intersection_ids_,
        current_line_intersection_points_, update_interval,
        confirmation_threshold, gate_threshold, min_gate_threshold,
        max_gate_threshold, prob_of_detection, prob_of_survival,
        clutter_intensity, initial_existence_probability);

    current_line_intersections_.resize(2, 0);
    current_intersection_ids_.resize(2, 0);

    line_intersection_tracker_.delete_tracks();

    if (debug_visualization_) {
        auto scene_update_line = visualize_track_gates(
            line_tracker_.get_tracks(), this->now(), target_frame_,
            gate_threshold, min_gate_threshold, max_gate_threshold, true,
            orca_pose_.position.z, altitude_);
        scene_update_line_pub_->publish(scene_update_line);

        auto scene_update_intersection = visualize_track_gates(
            line_intersection_tracker_.get_tracks(), this->now(), target_frame_,
            gate_threshold, min_gate_threshold, max_gate_threshold, false,
            orca_pose_.position.z, altitude_);

        auto marker_array =
            visualize_line_tracks(line_tracker_.get_tracks(), this->now(),
                                  target_frame_, orca_pose_.position.z,
                                  altitude_);
        line_points_pub_->publish(marker_array);
        scene_update_intersection_pub_->publish(scene_update_intersection);
    }

    // --- Per-cycle status summary --------------------------------------------
    {
        const auto& ltracks = line_tracker_.get_tracks();
        const auto& itracks = line_intersection_tracker_.get_tracks();
        int lconf = 0;
        for (const auto& t : ltracks) lconf += t.confirmed ? 1 : 0;
        int iconf = 0;
        for (const auto& t : itracks) iconf += t.confirmed ? 1 : 0;

        tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                          orca_pose_.orientation.z, orca_pose_.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        dlog("TICK: pose=(%.2f,%.2f) yaw=%.1fdeg alt=%.2f | line_tracks=%zu(conf=%d) "
             "int_tracks=%zu(conf=%d) used_intersections=%zu cur_line=%d cnt=%d",
             orca_pose_.position.x, orca_pose_.position.y, yaw * 180.0 / M_PI,
             altitude_, ltracks.size(), lconf, itracks.size(), iconf,
             used_line_intersections_.size(), current_line_id_,
             current_line_id_counter_);

        // Per-line-track detail
        for (const auto& t : ltracks) {
            dlog("  LINE id=%d conf=%d hits=%d p0=(%.2f,%.2f) p1=(%.2f,%.2f)",
                 t.id, t.confirmed ? 1 : 0, t.hits(), t.line_points(0, 0),
                 t.line_points(1, 0), t.line_points(0, 1), t.line_points(1, 1));
        }
        // Per-intersection-track detail
        for (const auto& t : itracks) {
            dlog("  INT  id=%d conf=%d hits=%d at=(%.2f,%.2f) parents=(%d,%d)",
                 t.id, t.confirmed ? 1 : 0, t.hits(), t.state.mean()(0),
                 t.state.mean()(1), t.id1, t.id2);
        }
    }

    if (new_intersection_available()) {
        dlog("BRANCH: new_intersection_available -> publish_intersection");
        publish_intersection();
        return;
    }
    if (line_tracker_.get_tracks().size() == 0) {
        dlog("BRANCH: no line tracks -> idle");
        return;
    }
    if (used_line_intersections_.size() == 0) {
        dlog("BRANCH: no used intersections -> find_and_publish_initial_waypoint");
        find_and_publish_initial_waypoint();
        return;
    }
    dlog("BRANCH: post-intersection -> termination_check + publish_waypoint");
    termination_check();
    if (used_line_intersections_.size() > 0) {
        publish_waypoint();
    }
}

void LineFilteringNode::publish_intersection() {
    for (const auto& int_track : line_intersection_tracker_.get_tracks()) {
        if (!int_track.confirmed) {
            continue;
        }
        set_next_line(int_track);

        const double ix = int_track.state.mean()(0);
        const double iy = int_track.state.mean()(1);

        dlog("PUBLISH_INT: junction id=%d at (%.2f,%.2f) parents=(%d,%d) -> "
             "next_line_id=%d next_yaw=%.1fdeg (cur_line=%d cnt=%d)",
             int_track.id, ix, iy, int_track.id1, int_track.id2, next_line_id_,
             next_line_yaw_ * 180.0 / M_PI, current_line_id_,
             current_line_id_counter_);

        used_line_intersections_.push_back(LineIntersection{
            ix, iy, int_track.id1, int_track.id2, int_track.line_points});

        // Two priority waypoints for the junction:
        //   1. Translate to the intersection while heading forward.
        //   2. Rotate in place to align with the outgoing line.
        auto wp_translate = make_waypoint(
            ix, iy, 0.0, vortex_msgs::msg::WaypointMode::FORWARD_HEADING);
        auto wp_align =
            make_waypoint(ix, iy, next_line_yaw_,
                          vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION);

        enqueue_waypoint(wp_translate, /*overwrite_prior=*/true,
                         /*take_priority=*/true, switching_threshold_);
        enqueue_waypoint(wp_align, /*overwrite_prior=*/false,
                         /*take_priority=*/true, switching_threshold_);

        // Remove so not used again
        line_intersection_tracker_.delete_track_by_id(int_track.id);

        geometry_msgs::msg::PointStamped intersection_point;
        intersection_point.header.frame_id = target_frame_;
        intersection_point.header.stamp = this->now();
        intersection_point.point.x = ix;
        intersection_point.point.y = iy;
        line_intersection_pub_->publish(intersection_point);

        if (debug_visualization_) {
            geometry_msgs::msg::PoseStamped intersection_pose;
            intersection_pose.header.frame_id = target_frame_;
            intersection_pose.header.stamp = this->now();
            intersection_pose.pose.position.x = ix;
            intersection_pose.pose.position.y = iy;
            intersection_pose.pose.position.z = orca_pose_.position.z;
            intersection_pose.pose.orientation = quat_from_yaw(next_line_yaw_);
            line_intersection_pose_pub_->publish(intersection_pose);
        }
        return;
    }
}

bool LineFilteringNode::new_intersection_available() {
    for (const auto& track : line_intersection_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }
        return true;
    }
    return false;
}

void LineFilteringNode::set_next_line(const Track& int_track) {
    // Get the yaw from the current orientation.
    tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                      orca_pose_.orientation.z, orca_pose_.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Get the endpoints of the line and the intersection point.
    Eigen::Vector2d line1 = int_track.line_points.col(0);
    Eigen::Vector2d line2 = int_track.line_points.col(1);
    Eigen::Vector2d intersection = int_track.state.mean();

    // Compute vectors from the line endpoints to the intersection.
    Eigen::Vector2d vec1 = (intersection - line1).normalized();  // id1
    Eigen::Vector2d vec2 = (intersection - line2).normalized();  // id2

    // Create the yaw direction vector.
    Eigen::Vector2d yaw_dir(std::cos(yaw), std::sin(yaw));

    // Compute the angles between the yaw direction and each vector.
    double dot1 = std::max(-1.0, std::min(1.0, yaw_dir.dot(vec1)));
    double dot2 = std::max(-1.0, std::min(1.0, yaw_dir.dot(vec2)));
    double angle1 = std::acos(dot1);
    double angle2 = std::acos(dot2);

    // First check: if we've already processed enough lines and one of the ids
    // matches, choose the "other" line.
    if (current_line_id_counter_ > 5 &&
        (current_line_id_ == int_track.id1 ||
         current_line_id_ == int_track.id2)) {
        if (current_line_id_ == int_track.id1) {
            next_line_id_ = int_track.id2;
            // Negate: vec2 points into the crossing; we want to head outward
            // along the arm toward its far endpoint.
            next_line_yaw_ = std::atan2(-vec2.y(), -vec2.x());
            dlog("SET_NEXT: OVERRIDE (came in on id1=%d) -> other arm id2=%d "
                 "yaw=%.1fdeg",
                 int_track.id1, int_track.id2, next_line_yaw_ * 180.0 / M_PI);
            return;
        } else if (current_line_id_ == int_track.id2) {
            next_line_id_ = int_track.id1;
            // Negate: vec1 points into the crossing; we want to head outward
            // along the arm toward its far endpoint.
            next_line_yaw_ = std::atan2(-vec1.y(), -vec1.x());
            dlog("SET_NEXT: OVERRIDE (came in on id2=%d) -> other arm id1=%d "
                 "yaw=%.1fdeg",
                 int_track.id2, int_track.id1, next_line_yaw_ * 180.0 / M_PI);
            return;
        }
    }

    // Otherwise, choose the candidate with the biggest angle difference.
    // Negate the chosen vec so next_line_yaw_ heads outward along the arm
    // (toward its far endpoint) rather than back into the crossing.
    if (angle1 > angle2) {
        next_line_id_ = int_track.id1;
        next_line_yaw_ = std::atan2(-vec1.y(), -vec1.x());
    } else {
        next_line_id_ = int_track.id2;
        next_line_yaw_ = std::atan2(-vec2.y(), -vec2.x());
    }
    dlog("SET_NEXT: ANGLE-FALLBACK angle1=%.1f angle2=%.1f (deg) -> next_id=%d "
         "yaw=%.1fdeg",
         angle1 * 180.0 / M_PI, angle2 * 180.0 / M_PI, next_line_id_,
         next_line_yaw_ * 180.0 / M_PI);
}

void LineFilteringNode::find_new_line_intersections() {
    // Define threshold for connected lines
    double connected_threshold =
        get_parameter("connected_lines_threshold").as_double();
    double min_angle = get_parameter("crossing_min_angle").as_double();
    std::vector<Track> tracks = line_tracker_.get_tracks();
    std::vector<LineIntersection> intersections;

    int confirmed_count = 0;
    for (const auto& t : tracks) confirmed_count += t.confirmed ? 1 : 0;
    dlog("FIND_INT: %zu line tracks (%d confirmed), conn_thresh=%.2f "
         "min_angle=%.2frad",
         tracks.size(), confirmed_count, connected_threshold, min_angle);

    for (const auto& track : tracks) {
        if (!track.confirmed) continue;

        for (const auto& track2 : tracks) {
            if (!track2.confirmed) continue;
            if (track.id == track2.id) continue;

            Eigen::Matrix<double, 2, 2> line1_points = track.line_points;
            Eigen::Matrix<double, 2, 2> line2_points = track2.line_points;
            Eigen::Vector2d intersection;

            if (!find_intersection(line1_points, line2_points, intersection,
                                   min_angle)) {
                dlog("  pair(%d,%d): no crossing (angle<min or parallel)",
                     track.id, track2.id);
                continue;
            }

            Eigen::Vector2d intersection_point_line1;
            Eigen::Vector2d intersection_point_line2;
            LineIntersection line_intersection;

            // Store points not used in intersection in line_points
            if ((intersection - line1_points.col(0)).norm() <
                (intersection - line1_points.col(1)).norm()) {
                intersection_point_line1 = line1_points.col(0);
                line_intersection.line_points.col(0) << line1_points.col(1);
            } else {
                intersection_point_line1 = line1_points.col(1);
                line_intersection.line_points.col(0) << line1_points.col(0);
            }

            if ((intersection - line2_points.col(0)).norm() <
                (intersection - line2_points.col(1)).norm()) {
                intersection_point_line2 = line2_points.col(0);
                line_intersection.line_points.col(1) << line2_points.col(1);
            } else {
                intersection_point_line2 = line2_points.col(1);
                line_intersection.line_points.col(1) << line2_points.col(0);
            }

            const double conn =
                (intersection_point_line1 - intersection_point_line2).norm();
            if (conn < connected_threshold) {
                line_intersection.x = intersection(0);
                line_intersection.y = intersection(1);
                line_intersection.id1 = track.id;
                line_intersection.id2 = track2.id;

                // Skip intersections already traversed.
                if (std::find(used_line_intersections_.begin(),
                              used_line_intersections_.end(),
                              line_intersection) !=
                    used_line_intersections_.end()) {
                    dlog("  pair(%d,%d): crossing at (%.2f,%.2f) conn=%.2f "
                         "-> SKIPPED (already used)",
                         track.id, track2.id, intersection(0), intersection(1),
                         conn);
                    continue;
                }
                dlog("  pair(%d,%d): crossing at (%.2f,%.2f) conn=%.2f "
                     "-> CANDIDATE",
                     track.id, track2.id, intersection(0), intersection(1),
                     conn);
                intersections.push_back(line_intersection);
            } else {
                dlog("  pair(%d,%d): crossing at (%.2f,%.2f) but conn=%.2f >= "
                     "%.2f -> rejected (arms not connected)",
                     track.id, track2.id, intersection(0), intersection(1),
                     conn, connected_threshold);
            }
        }
    }
    int size = intersections.size();
    dlog("FIND_INT: %d intersection candidates fed to tracker", size);
    current_line_intersections_ =
        Eigen::Array<double, 2, Eigen::Dynamic>(2, size);
    current_intersection_ids_ = Eigen::Array<int, 2, Eigen::Dynamic>(2, size);
    current_line_intersection_points_ =
        Eigen::Array<double, 2, Eigen::Dynamic>(2, 2 * size);

    for (size_t i = 0; i < intersections.size(); i++) {
        current_line_intersections_.col(i)
            << intersections.at(i).x, intersections.at(i).y;
        current_intersection_ids_.col(i)
            << intersections.at(i).id1, intersections.at(i).id2;
        current_line_intersection_points_.col(2 * i)
            << intersections.at(i).line_points.col(0);
        current_line_intersection_points_.col(2 * i + 1)
            << intersections.at(i).line_points.col(1);
    }
}

bool LineFilteringNode::find_intersection(
    const Eigen::Matrix<double, 2, 2>& line1,
    const Eigen::Matrix<double, 2, 2>& line2, Eigen::Vector2d& intersection,
    double min_angle) {
    double x1 = line1(0, 0), y1 = line1(1, 0);
    double x2 = line1(0, 1), y2 = line1(1, 1);

    double x3 = line2(0, 0), y3 = line2(1, 0);
    double x4 = line2(0, 1), y4 = line2(1, 1);

    Eigen::Vector2d dir1(x2 - x1, y2 - y1);
    Eigen::Vector2d dir2(x4 - x3, y4 - y3);

    double dot_product = dir1.dot(dir2);
    double mag1 = dir1.norm();
    double mag2 = dir2.norm();

    double cos_theta = dot_product / (mag1 * mag2);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    double angle_rad = std::acos(cos_theta);

    if (angle_rad < min_angle) {
        return false;
    }

    double m1 = (y2 - y1) / (x2 - x1);
    double m2 = (y4 - y3) / (x4 - x3);

    if (m1 == m2) {
        return false;
    }

    double x = (y3 - y1 + m1 * x1 - m2 * x3) / (m1 - m2);
    double y = m1 * (x - x1) + y1;

    intersection << x, y;
    return true;
}

int LineFilteringNode::get_track_by_id(Track& line_track, int id) {
    for (const auto& track : line_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }
        if (track.id == id) {
            line_track = track;
            return id;
        }
    }
    return -1;
}

bool LineFilteringNode::follow_toward(double target_x, double target_y) {
    // Current heading.
    tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                      orca_pose_.orientation.z, orca_pose_.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Bearing from the vehicle to the target endpoint.
    // NOTE: do NOT overwrite next_line_yaw_ here. next_line_yaw_ holds the
    // junction's intended outgoing-line direction (set in set_next_line) and is
    // used by get_track_by_yaw to recover the correct line if its track flickers.
    // Clobbering it with the per-cycle bearing made recovery lock onto the wrong
    // (incoming) line after a junction.
    const double desired_yaw = std::atan2(target_y - orca_pose_.position.y,
                                          target_x - orca_pose_.position.x);

    const double heading_err = std::fabs(
        std::atan2(std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw)));

    dlog("FOLLOW: target=(%.2f,%.2f) desired_yaw=%.1fdeg cur_yaw=%.1fdeg "
         "err=%.1fdeg",
         target_x, target_y, desired_yaw * 180.0 / M_PI, yaw * 180.0 / M_PI,
         heading_err * 180.0 / M_PI);

    // Always drive with FORWARD_HEADING: the controller turns toward the target
    // while translating, which handles corners quickly. (A standalone
    // ONLY_ORIENTATION "rotate in place first" barely turns the vehicle ~0.1
    // deg/cycle and stalls at junctions, so it is intentionally not used here.)
    //
    // Throttle the resend: skip if the target has not moved enough, so we don't
    // wipe the manager queue / reset the reference filter every cycle. Force one
    // through after max_resend_skips_ consecutive skips.
    if (have_prev_wp_) {
        const double moved = std::hypot(target_x - prev_wp_x_,
                                        target_y - prev_wp_y_);
        if (moved < min_wp_dist_) {
            if (++wp_skip_count_ < max_resend_skips_) {
                dlog("FOLLOW: FORWARD skipped (moved=%.2f < %.2f, skip %d/%d)",
                     moved, min_wp_dist_, wp_skip_count_, max_resend_skips_);
                return true;  // treat as still-following toward the same target
            }
            wp_skip_count_ = 0;
        } else {
            wp_skip_count_ = 0;
        }
    }

    dlog("FOLLOW: SEND FORWARD_HEADING to (%.2f,%.2f)", target_x, target_y);
    auto wp = make_waypoint(target_x, target_y, 0.0,
                            vortex_msgs::msg::WaypointMode::FORWARD_HEADING);
    enqueue_waypoint(wp, /*overwrite_prior=*/true, /*take_priority=*/false,
                     switching_threshold_);
    prev_wp_x_ = target_x;
    prev_wp_y_ = target_y;
    prev_wp_yaw_ = desired_yaw;
    prev_wp_mode_ = vortex_msgs::msg::WaypointMode::FORWARD_HEADING;
    have_prev_wp_ = true;
    return true;
}

void LineFilteringNode::find_and_publish_initial_waypoint() {
    geometry_msgs::msg::Pose Pose_orca = orca_pose_;
    double orca_x = Pose_orca.position.x;
    double orca_y = Pose_orca.position.y;

    tf2::Quaternion q(Pose_orca.orientation.x, Pose_orca.orientation.y,
                      Pose_orca.orientation.z, Pose_orca.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    auto tracks = line_tracker_.get_tracks();

    Track chosen_track;

    if (tracks.size() == 0) {
        return;
    }

    if (tracks.size() == 1) {
        chosen_track = tracks.front();
    }

    if (tracks.size() > 1) {
        double min_distance = std::numeric_limits<double>::max();
        chosen_track = tracks.front();

        double rx = orca_x;
        double ry = orca_y;

        for (const auto& track : tracks) {
            Eigen::Vector2d a = track.line_points.col(0);
            Eigen::Vector2d b = track.line_points.col(1);

            double ax = a(0);
            double ay = a(1);
            double bx = b(0);
            double by = b(1);

            double ABx = bx - ax;
            double ABy = by - ay;
            double normAB = std::hypot(ABx, ABy);
            if (normAB < 1e-6) {
                continue;
            }

            double ARx = rx - ax;
            double ARy = ry - ay;
            double cross_mag = std::fabs(ABx * ARy - ABy * ARx);
            double distance = cross_mag / normAB;

            if (distance < min_distance) {
                min_distance = distance;
                chosen_track = track;
            }
        }
    }

    double rx = orca_x;
    double ry = orca_y;

    double ax = chosen_track.line_points(0, 0);
    double ay = chosen_track.line_points(1, 0);
    double bx = chosen_track.line_points(0, 1);
    double by = chosen_track.line_points(1, 1);

    double ABx = bx - ax;
    double ABy = by - ay;
    double ARx = rx - ax;
    double ARy = ry - ay;

    double ab_squared = ABx * ABx + ABy * ABy;
    double t = (ab_squared == 0.0 ? 0.0
                                  : ((ARx * ABx + ARy * ABy) / ab_squared));

    bool robotBetween = (t >= 0.0 && t <= 1.0);

    double chosen_x = 0.0;
    double chosen_y = 0.0;

    if (robotBetween) {
        double dx1 = ax - rx;
        double dy1 = ay - ry;
        double dx2 = bx - rx;
        double dy2 = by - ry;

        double angle1 = std::atan2(dy1, dx1);
        double angle2 = std::atan2(dy2, dx2);

        double diff1 = std::fabs(
            std::atan2(std::sin(angle1 - yaw), std::cos(angle1 - yaw)));
        double diff2 = std::fabs(
            std::atan2(std::sin(angle2 - yaw), std::cos(angle2 - yaw)));

        if (diff1 < diff2) {
            chosen_x = ax;
            chosen_y = ay;
        } else {
            chosen_x = bx;
            chosen_y = by;
        }
    } else {
        double dx1 = ax - rx;
        double dy1 = ay - ry;
        double dx2 = bx - rx;
        double dy2 = by - ry;

        double dist1 = std::hypot(dx1, dy1);
        double dist2 = std::hypot(dx2, dy2);

        if (dist1 > dist2) {
            chosen_x = ax;
            chosen_y = ay;
        } else {
            chosen_x = bx;
            chosen_y = by;
        }
    }

    if (current_line_id_ == chosen_track.id) {
        current_line_id_counter_++;
    } else {
        current_line_id_ = chosen_track.id;
        current_line_id_counter_ = 0;
    }

    dlog("INITIAL_WP: chosen_line=%d robotBetween=%d -> endpoint=(%.2f,%.2f)",
         chosen_track.id, robotBetween ? 1 : 0, chosen_x, chosen_y);
    // Drive toward the chosen endpoint, rotating to face it first if needed.
    follow_toward(chosen_x, chosen_y);

    geometry_msgs::msg::PointStamped chosen_point;
    chosen_point.header.stamp = this->now();
    chosen_point.header.frame_id = target_frame_;
    chosen_point.point.x = chosen_x;
    chosen_point.point.y = chosen_y;
    chosen_point.point.z = orca_pose_.position.z;
    line_point_pub_->publish(chosen_point);

    if (debug_visualization_) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = target_frame_;
        pose_msg.pose.position.x = chosen_x;
        pose_msg.pose.position.y = chosen_y;
        pose_msg.pose.position.z = orca_pose_.position.z;
        line_pose_pub_->publish(pose_msg);
    }
}

void LineFilteringNode::publish_waypoint() {
    auto intersection = used_line_intersections_.back();

    Track next_line;
    if (get_track_by_id(next_line, next_line_id_) == -1) {
        dlog("PUB_WP: next_line_id=%d NOT found -> get_track_by_yaw fallback "
             "(target_yaw=%.1fdeg)",
             next_line_id_, next_line_yaw_ * 180.0 / M_PI);
        get_track_by_yaw(next_line);
    } else {
        dlog("PUB_WP: following next_line_id=%d", next_line_id_);
    }

    // Guard: if neither lookup found a confirmed track, next_line is still the
    // default-constructed Track (id=-1, zero endpoints). Do NOT issue a waypoint
    // -- otherwise we'd command FORWARD_HEADING to odom origin (0,0). Hold until
    // a confirmed line reappears.
    if (next_line.id == -1) {
        dlog("PUB_WP: no confirmed next line found -> holding (no waypoint)");
        return;
    }

    if (current_line_id_ == next_line.id) {
        current_line_id_counter_++;
    } else {
        current_line_id_ = next_line.id;
        current_line_id_counter_ = 0;
    }

    Eigen::Vector2d next_line_p1 = next_line.line_points.col(0);
    Eigen::Vector2d next_line_p2 = next_line.line_points.col(1);
    double distance1 = std::hypot(intersection.x - next_line_p1(0),
                                  intersection.y - next_line_p1(1));
    double distance2 = std::hypot(intersection.x - next_line_p2(0),
                                  intersection.y - next_line_p2(1));

    double chosen_x, chosen_y;
    if (distance1 > distance2) {
        chosen_x = next_line_p1(0);
        chosen_y = next_line_p1(1);
    } else {
        chosen_x = next_line_p2(0);
        chosen_y = next_line_p2(1);
    }

    dlog("PUB_WP: next_line=%d endpoints p1=(%.2f,%.2f) p2=(%.2f,%.2f) "
         "junction=(%.2f,%.2f) -> far endpoint=(%.2f,%.2f)",
         next_line.id, next_line_p1(0), next_line_p1(1), next_line_p2(0),
         next_line_p2(1), intersection.x, intersection.y, chosen_x, chosen_y);

    // Rotate to face the (possibly newly chosen) outgoing line before driving
    // forward, so a re-selected line far off the current heading does not
    // produce a curved approach.
    follow_toward(chosen_x, chosen_y);

    geometry_msgs::msg::PointStamped line_point;
    line_point.header.frame_id = target_frame_;
    line_point.header.stamp = this->now();
    line_point.point.x = chosen_x;
    line_point.point.y = chosen_y;
    line_point.point.z = orca_pose_.position.z;
    line_point_pub_->publish(line_point);

    if (debug_visualization_) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = target_frame_;
        pose_msg.pose.position.x = chosen_x;
        pose_msg.pose.position.y = chosen_y;
        pose_msg.pose.position.z = orca_pose_.position.z;
        line_pose_pub_->publish(pose_msg);
    }
}

void LineFilteringNode::get_track_by_yaw(Track& line_track) {
    double prev_yaw = next_line_yaw_;
    double min_diff = std::numeric_limits<double>::max();
    for (const auto& track : line_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }
        Eigen::Vector2d a = track.line_points.col(0);
        Eigen::Vector2d b = track.line_points.col(1);
        double angle = std::atan2(b(1) - a(1), b(0) - a(0));
        // Lines are undirected -> compare orientation modulo pi so the
        // detector's endpoint ordering can't flip the match.
        double diff = std::fmod(std::fabs(angle - prev_yaw), M_PI);
        if (diff > M_PI / 2.0) {
            diff = M_PI - diff;
        }
        if (diff < min_diff) {
            min_diff = diff;
            line_track = track;
        }
    }
}

void LineFilteringNode::termination_check() {
    Track line_track;
    if (get_track_by_id(line_track, current_line_id_) == -1) {
        termination_counter_ = 0;
        return;
    }
    Eigen::Vector2d line_p1 = line_track.line_points.col(0);
    Eigen::Vector2d line_p2 = line_track.line_points.col(1);
    auto intersection = used_line_intersections_.back();
    Eigen::Vector2d orca_position(orca_pose_.position.x, orca_pose_.position.y);
    double distance1 = std::hypot(intersection.x - line_p1(0),
                                  intersection.y - line_p1(1));
    double distance2 = std::hypot(intersection.x - line_p2(0),
                                  intersection.y - line_p2(1));
    Eigen::Vector2d endpoint = (distance1 > distance2) ? line_p1 : line_p2;
    double distance = std::hypot(orca_position(0) - endpoint(0),
                                 orca_position(1) - endpoint(1));
    if (distance < 0.5) {
        termination_counter_++;
    } else {
        termination_counter_ = 0;
    }
    if (termination_counter_ >
        this->get_parameter("termination_counter_threshold").as_int()) {
        RCLCPP_INFO(this->get_logger(),
                    "End of pipeline reached -- notifying FSM.");
        is_executing_action_ = false;
        termination_counter_ = 0;
        if (finished_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            finished_client_->async_send_request(req);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "pipeline_finished service not available.");
        }
    }
}

void LineFilteringNode::update_timer(int update_interval) {
    timer_->cancel();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_interval),
        std::bind(&LineFilteringNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Updated timer with %d ms update interval",
                update_interval);
}

void LineFilteringNode::update_dyn_model(double std_dynmod) {
    line_tracker_.set_dyn_model(std_dynmod);
    RCLCPP_INFO(this->get_logger(), "Updated dynamic model");
}

void LineFilteringNode::update_sensor_model(double std_measurement) {
    line_tracker_.set_sensor_model(std_measurement);
    RCLCPP_INFO(this->get_logger(), "Updated sensor model");
}
