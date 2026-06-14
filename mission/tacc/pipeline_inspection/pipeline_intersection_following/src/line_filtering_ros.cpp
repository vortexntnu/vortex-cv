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

    declare_parameter<double>("lookahead_distance", 2.0);

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
    camera_frame_ = this->declare_parameter<std::string>(
        "camera_frame", "nautilus/downwards_camera_optical");
    auto lines_sub_topic = this->declare_parameter<std::string>(
        "lines_sub_topic", "/irls_line/lines");
    auto camera_info_sub_topic = this->declare_parameter<std::string>(
        "camera_info_sub_topic", "/pipeline/down_camera/camera_info");
    auto altitude_sub_topic = this->declare_parameter<std::string>(
        "altitude_sub_topic", "/nautilus/dvl/altitude");
    auto odom_sub_topic = this->declare_parameter<std::string>(
        "odom_sub_topic", "/nautilus/odom");

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

    // --- TF
    // -------------------------------------------------------------------
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // --- Inputs
    // ---------------------------------------------------------------
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

    // --- Debug visualization
    // --------------------------------------------------
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
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/scene_update", qos_sensor_data);
        scene_update_intersection_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/scene_update_intersection", qos_sensor_data);
        line_intersection_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/line/intersection_pose", qos_sensor_data);
        line_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/line/pose", qos_sensor_data);
        termination_track_count_pub_ =
            this->create_publisher<std_msgs::msg::Int32>(
                "/line/termination_track_count", qos_sensor_data);
    }

    // --- Track managers
    // -------------------------------------------------------
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

    // --- Timer
    // ----------------------------------------------------------------
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
        junction_votes_.clear();
        termination_counter_ = 0;
        current_line_id_ = -1;
        current_line_id_counter_ = 0;
        next_line_yaw_ = 0.0;
        have_prev_wp_ = false;
        wp_skip_count_ = 0;
        prev_wp_mode_ = 255;
        junction_in_progress_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Pipeline following stopped");
    }
    response->success = is_executing_action_;
    response->message = is_executing_action_ ? "following" : "stopped";
}

void LineFilteringNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    dlog("CAM_INFO: K initialized fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
         K_.at<double>(0, 0), K_.at<double>(1, 1),
         K_.at<double>(0, 2), K_.at<double>(1, 2));
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
                    case 0:
                        point_1_->publish(pose_msg);
                        break;
                    case 1:
                        point_2_->publish(pose_msg);
                        break;
                    case 2:
                        point_3_->publish(pose_msg);
                        break;
                    case 3:
                        point_4_->publish(pose_msg);
                        break;
                    default:
                        break;
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
            const double ang =
                std::atan2(measurements_(1, e1) - measurements_(1, e0),
                           measurements_(0, e1) - measurements_(0, e0));
            dlog(
                "  MEAS line %ld: mid=(%.2f,%.2f) ang=%.1fdeg "
                "p0=(%.2f,%.2f) p1=(%.2f,%.2f)",
                (long)c, line_params_(0, c), line_params_(1, c),
                ang * 180.0 / M_PI, measurements_(0, e0), measurements_(1, e0),
                measurements_(0, e1), measurements_(1, e1));
        }
        if (ncols == 2) {
            const double sep =
                std::hypot(line_params_(0, 0) - line_params_(0, 1),
                           line_params_(1, 0) - line_params_(1, 1));
            dlog(
                "  MEAS midpoint separation = %.2f m (min_gate=%.2f, "
                "max_gate=%.2f)",
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
    double x,
    double y,
    double yaw,
    uint8_t mode) const {
    vortex_msgs::msg::Waypoint wp;
    wp.pose.position.x = x;
    wp.pose.position.y = y;
    wp.pose.position.z = orca_pose_.position.z;
    wp.pose.orientation = quat_from_yaw(yaw);
    wp.waypoint_mode.mode = mode;
    // Hold altitude for translating waypoints, but not for orientation-only
    // ones
    // -- commanding altitude there can destabilise heading convergence.
    wp.keep_altitude =
        (mode != vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION);
    wp.desired_altitude = target_altitude_;
    return wp;
}

void LineFilteringNode::enqueue_waypoint(const vortex_msgs::msg::Waypoint& wp,
                                         bool overwrite_prior,
                                         bool take_priority,
                                         double switching_threshold) {
    {
        tf2::Quaternion q(wp.pose.orientation.x, wp.pose.orientation.y,
                          wp.pose.orientation.z, wp.pose.orientation.w);
        double r, p, wp_yaw;
        tf2::Matrix3x3(q).getRPY(r, p, wp_yaw);
        dlog("ENQUEUE_WP: x=%.2f y=%.2f yaw=%.1fdeg mode=%d overwrite=%d priority=%d thresh=%.2f queue_size=%zu",
             wp.pose.position.x, wp.pose.position.y, wp_yaw * 180.0 / M_PI,
             wp.waypoint_mode.mode,
             overwrite_prior ? 1 : 0, take_priority ? 1 : 0,
             switching_threshold, request_queue_.size());
    }
    request_queue_.push_back(
        {wp, overwrite_prior, take_priority, switching_threshold});
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

    dlog("SEND_WP: x=%.2f y=%.2f mode=%d overwrite=%d priority=%d remaining_queue=%zu",
         item.wp.pose.position.x, item.wp.pose.position.y,
         item.wp.waypoint_mode.mode,
         item.overwrite_prior ? 1 : 0, item.take_priority ? 1 : 0,
         request_queue_.size());

    waypoint_client_->async_send_request(
        req,
        [this](rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedFuture
                   future) {
            request_in_flight_ = false;
            try {
                auto resp = future.get();
                dlog("SEND_WP_RESP: success=%d", resp->success ? 1 : 0);
            } catch (...) {
                dlog("SEND_WP_RESP: FAILED (exception)");
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

    // Snapshot track state before update so we can log changes.
    struct TrackSnap { int id; bool confirmed; };
    std::vector<TrackSnap> pre_snap;
    for (const auto& t : line_tracker_.get_tracks())
        pre_snap.push_back({t.id, t.confirmed});

    // Update line tracks
    line_tracker_.update_line_tracks(
        measurements_, line_params_, update_interval, confirmation_threshold,
        gate_threshold, min_gate_threshold, max_gate_threshold,
        prob_of_detection, prob_of_survival, clutter_intensity,
        initial_existence_probability);

    measurements_.resize(2, 0);
    line_params_.resize(2, 0);

    // Log newly created or newly confirmed tracks.
    for (const auto& t : line_tracker_.get_tracks()) {
        auto it = std::find_if(pre_snap.begin(), pre_snap.end(),
                               [&](const TrackSnap& s){ return s.id == t.id; });
        if (it == pre_snap.end()) {
            double ang = std::atan2(t.line_points(1,1) - t.line_points(1,0),
                                    t.line_points(0,1) - t.line_points(0,0));
            dlog("TRACK_NEW: id=%d p0=(%.2f,%.2f) p1=(%.2f,%.2f) ang=%.1fdeg",
                 t.id, t.line_points(0,0), t.line_points(1,0),
                 t.line_points(0,1), t.line_points(1,1), ang * 180.0 / M_PI);
        } else if (!it->confirmed && t.confirmed) {
            dlog("TRACK_CONF: id=%d p0=(%.2f,%.2f) p1=(%.2f,%.2f)",
                 t.id, t.line_points(0,0), t.line_points(1,0),
                 t.line_points(0,1), t.line_points(1,1));
        }
    }

    // delete tracks
    line_tracker_.delete_tracks();

    // Log deletions.
    const auto& post_tracks = line_tracker_.get_tracks();
    for (const auto& s : pre_snap) {
        bool still_alive = std::any_of(post_tracks.begin(), post_tracks.end(),
                                       [&](const Track& t){ return t.id == s.id; });
        if (!still_alive)
            dlog("TRACK_DEL: id=%d (was confirmed=%d)", s.id, s.confirmed ? 1 : 0);
    }

    // Update junction vote counters (replaces second IPDA tracker)
    find_new_line_intersections();

    if (debug_visualization_) {
        auto scene_update_line = visualize_track_gates(
            line_tracker_.get_tracks(), this->now(), target_frame_,
            gate_threshold, min_gate_threshold, max_gate_threshold, true,
            orca_pose_.position.z, altitude_);
        scene_update_line_pub_->publish(scene_update_line);

        auto scene_update_intersection = visualize_track_gates(
            {}, this->now(), target_frame_, gate_threshold, min_gate_threshold,
            max_gate_threshold, false, orca_pose_.position.z, altitude_);

        auto marker_array = visualize_line_tracks(
            line_tracker_.get_tracks(), this->now(), target_frame_,
            orca_pose_.position.z, altitude_);
        line_points_pub_->publish(marker_array);
        scene_update_intersection_pub_->publish(scene_update_intersection);
    }

    // --- Per-cycle status summary --------------------------------------------
    {
        const auto& ltracks = line_tracker_.get_tracks();
        int lconf = 0;
        for (const auto& t : ltracks)
            lconf += t.confirmed ? 1 : 0;
        int ready_votes = 0;
        for (const auto& v : junction_votes_)
            ready_votes += (v.hits >= kJunctionConfirmHits) ? 1 : 0;

        tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                          orca_pose_.orientation.z, orca_pose_.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        dlog(
            "TICK: pose=(%.2f,%.2f) yaw=%.1fdeg alt=%.2f | "
            "line_tracks=%zu(conf=%d) "
            "votes=%zu(ready=%d) used_junctions=%zu cur_line=%d cnt=%d",
            orca_pose_.position.x, orca_pose_.position.y, yaw * 180.0 / M_PI,
            altitude_, ltracks.size(), lconf, junction_votes_.size(),
            ready_votes, used_line_intersections_.size(), current_line_id_,
            current_line_id_counter_);

        for (const auto& t : ltracks) {
            dlog("  LINE id=%d conf=%d hits=%d p0=(%.2f,%.2f) p1=(%.2f,%.2f)",
                 t.id, t.confirmed ? 1 : 0, t.hits(), t.line_points(0, 0),
                 t.line_points(1, 0), t.line_points(0, 1), t.line_points(1, 1));
        }
        for (const auto& v : junction_votes_) {
            dlog("  VOTE pair=(%d,%d) hits=%d/%d at=(%.2f,%.2f)", v.id1, v.id2,
                 v.hits, kJunctionConfirmHits, v.pos(0), v.pos(1));
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
        dlog(
            "BRANCH: no used intersections -> "
            "find_and_publish_initial_waypoint");
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
    for (const auto& vote : junction_votes_) {
        if (vote.hits < kJunctionConfirmHits)
            continue;

        set_next_line(vote);

        // Capture vehicle yaw before the turn (used for logging).
        {
            tf2::Quaternion q_v(
                orca_pose_.orientation.x, orca_pose_.orientation.y,
                orca_pose_.orientation.z, orca_pose_.orientation.w);
            double r, p;
            tf2::Matrix3x3(q_v).getRPY(r, p, pre_junction_yaw_);
        }

        const double ix = vote.pos(0);
        const double iy = vote.pos(1);

        dlog(
            "PUBLISH_INT: junction pair=(%d,%d) at (%.2f,%.2f) hits=%d -> "
            "next_yaw=%.1fdeg pre_junction_yaw=%.1fdeg (cur_line=%d cnt=%d)",
            vote.id1, vote.id2, ix, iy, vote.hits,
            next_line_yaw_ * 180.0 / M_PI, pre_junction_yaw_ * 180.0 / M_PI,
            current_line_id_, current_line_id_counter_);

        used_line_intersections_.push_back(
            LineIntersection{ix, iy, vote.id1, vote.id2, vote.line_points});

        // Send baselink to the raw junction position. Camera offset is not
        // applied here: the junction fires when the camera is already at the
        // corner, so a camera-adjusted WP would land at the robot's current
        // position and release instantly without any turn.
        double jx = ix, jy = iy;
        dlog("PUBLISH_INT: junction wp at (%.2f,%.2f)", jx, jy);

        auto wp_junction = make_waypoint(
            jx, jy, next_line_yaw_, vortex_msgs::msg::WaypointMode::XY_AND_YAW);

        enqueue_waypoint(wp_junction, /*overwrite_prior=*/true,
                         /*take_priority=*/true, switching_threshold_);

        junction_in_progress_ = true;
        junction_hold_ticks_ = 0;
        junction_wp_x_ = jx;
        junction_wp_y_ = jy;
        current_line_id_ = -1;  // Release sticky lock; yaw search selects outgoing track

        // Consume all votes — used_line_intersections_ position check prevents
        // the same corner being re-detected on the next cycle.
        junction_votes_.clear();

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
    if (junction_in_progress_) {
        dlog("NEW_INT: suppressed — junction in progress");
        return false;
    }
    for (const auto& v : junction_votes_) {
        if (v.hits >= kJunctionConfirmHits)
            return true;
    }
    return false;
}

void LineFilteringNode::set_next_line(const JunctionVote& vote) {
    // Reference direction: use the committed outgoing yaw from the last
    // junction (stable, set by angle geometry). At the first junction, fall
    // back to the current vehicle heading since no prior direction is
    // committed.
    double ref_yaw;
    if (used_line_intersections_.empty()) {
        tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                          orca_pose_.orientation.z, orca_pose_.orientation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, ref_yaw);
    } else {
        ref_yaw = next_line_yaw_;
    }

    // vote.line_points.col(0/1) = far endpoints of arm1/arm2, i.e. the
    // endpoints furthest from the crossing.  vec_inward points FROM the far
    // endpoint TOWARD the junction.  For the incoming arm (we approached from
    // its far end) that vector aligns with our heading → angle ≈ 0°.  For the
    // outgoing arm the vector opposes the new direction → angle is large.
    // Picking the arm with the largest inward-vector angle identifies the
    // outgoing arm unambiguously.
    Eigen::Vector2d far1 = vote.line_points.col(0);
    Eigen::Vector2d far2 = vote.line_points.col(1);
    Eigen::Vector2d vec1 = (vote.pos - far1).normalized();
    Eigen::Vector2d vec2 = (vote.pos - far2).normalized();

    Eigen::Vector2d ref_dir(std::cos(ref_yaw), std::sin(ref_yaw));
    double angle1 = std::acos(std::max(-1.0, std::min(1.0, ref_dir.dot(vec1))));
    double angle2 = std::acos(std::max(-1.0, std::min(1.0, ref_dir.dot(vec2))));

    // Outgoing yaw = direction away from junction along the outgoing arm.
    if (angle1 > angle2) {
        next_line_yaw_ = std::atan2(-vec1.y(), -vec1.x());
    } else {
        next_line_yaw_ = std::atan2(-vec2.y(), -vec2.x());
    }
    dlog(
        "SET_NEXT: pair=(%d,%d) angle1=%.1fdeg angle2=%.1fdeg ref=%.1fdeg -> "
        "next_yaw=%.1fdeg",
        vote.id1, vote.id2, angle1 * 180.0 / M_PI, angle2 * 180.0 / M_PI,
        ref_yaw * 180.0 / M_PI, next_line_yaw_ * 180.0 / M_PI);
}

void LineFilteringNode::find_new_line_intersections() {
    // Don't accumulate votes while the robot is executing a junction waypoint.
    // Votes built up during the hold would fire immediately when the hold
    // releases, causing the same corner to re-trigger and oscillate.
    if (junction_in_progress_) {
        junction_votes_.clear();
        return;
    }

    // Define threshold for connected lines
    double connected_threshold =
        get_parameter("connected_lines_threshold").as_double();
    double min_angle = get_parameter("crossing_min_angle").as_double();
    const std::vector<Track> tracks = line_tracker_.get_tracks();

    int confirmed_count = 0;
    for (const auto& t : tracks)
        confirmed_count += t.confirmed ? 1 : 0;
    dlog(
        "FIND_INT: %zu line tracks (%d confirmed), conn_thresh=%.2f "
        "min_angle=%.2frad",
        tracks.size(), confirmed_count, connected_threshold, min_angle);

    // Track which (id1,id2) pairs are active this tick so we can expire stale
    // votes.
    std::vector<std::pair<int, int>> active_pairs;

    for (const auto& t1 : tracks) {
        if (!t1.confirmed)
            continue;
        for (const auto& t2 : tracks) {
            if (!t2.confirmed)
                continue;
            if (t1.id >= t2.id)
                continue;  // unique unordered pairs only

            Eigen::Vector2d crossing;
            if (!find_intersection(t1.line_points, t2.line_points, crossing,
                                   min_angle)) {
                dlog("  pair(%d,%d): no crossing (angle<min or parallel)",
                     t1.id, t2.id);
                continue;
            }

            // Near endpoint = closest to crossing; far endpoint used for arm
            // direction.
            int near1 = (t1.line_points.col(0) - crossing).norm() <
                                (t1.line_points.col(1) - crossing).norm()
                            ? 0
                            : 1;
            int near2 = (t2.line_points.col(0) - crossing).norm() <
                                (t2.line_points.col(1) - crossing).norm()
                            ? 0
                            : 1;
            double conn =
                (t1.line_points.col(near1) - t2.line_points.col(near2)).norm();

            if (conn >= connected_threshold) {
                dlog(
                    "  pair(%d,%d): crossing at (%.2f,%.2f) conn=%.2f >= %.2f "
                    "-> rejected",
                    t1.id, t2.id, crossing(0), crossing(1), conn,
                    connected_threshold);
                continue;
            }

            // Skip if this crossing position is already in
            // used_line_intersections_.
            LineIntersection probe;
            probe.x = crossing(0);
            probe.y = crossing(1);
            probe.id1 = t1.id;
            probe.id2 = t2.id;
            if (std::find(used_line_intersections_.begin(),
                          used_line_intersections_.end(),
                          probe) != used_line_intersections_.end()) {
                dlog(
                    "  pair(%d,%d): crossing at (%.2f,%.2f) -> SKIPPED "
                    "(already used)",
                    t1.id, t2.id, crossing(0), crossing(1));
                continue;
            }

            active_pairs.push_back({t1.id, t2.id});

            // Build far-endpoint matrix for arm direction (used by
            // set_next_line).
            Eigen::Matrix<double, 2, 2> far_pts;
            far_pts.col(0) = t1.line_points.col(1 - near1);
            far_pts.col(1) = t2.line_points.col(1 - near2);

            // Update existing vote or create a new one.
            auto it =
                std::find_if(junction_votes_.begin(), junction_votes_.end(),
                             [&](const JunctionVote& v) {
                                 return v.id1 == t1.id && v.id2 == t2.id;
                             });

            if (it != junction_votes_.end()) {
                it->hits++;
                it->pos = crossing;  // latest position (line tracker smooths)
                it->line_points = far_pts;
                dlog(
                    "  pair(%d,%d): crossing at (%.2f,%.2f) conn=%.2f "
                    "hits=%d/%d",
                    t1.id, t2.id, crossing(0), crossing(1), conn, it->hits,
                    kJunctionConfirmHits);
            } else {
                junction_votes_.push_back({t1.id, t2.id, 1, crossing, far_pts});
                dlog(
                    "  pair(%d,%d): crossing at (%.2f,%.2f) conn=%.2f -> new "
                    "vote (hits=1/%d)",
                    t1.id, t2.id, crossing(0), crossing(1), conn,
                    kJunctionConfirmHits);
            }
        }
    }

    // Expire votes whose track pair is no longer active this tick.
    junction_votes_.erase(
        std::remove_if(
            junction_votes_.begin(), junction_votes_.end(),
            [&](const JunctionVote& v) {
                bool alive = std::find(active_pairs.begin(), active_pairs.end(),
                                       std::make_pair(v.id1, v.id2)) !=
                             active_pairs.end();
                if (!alive)
                    dlog("  pair(%d,%d): vote expired (pair no longer active)",
                         v.id1, v.id2);
                return !alive;
            }),
        junction_votes_.end());
}

bool LineFilteringNode::find_intersection(
    const Eigen::Matrix<double, 2, 2>& line1,
    const Eigen::Matrix<double, 2, 2>& line2,
    Eigen::Vector2d& intersection,
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

bool LineFilteringNode::follow_toward(double target_x,
                                      double target_y,
                                      double pipe_yaw) {
    // Current heading.
    tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                      orca_pose_.orientation.z, orca_pose_.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const double heading_err = std::fabs(
        std::atan2(std::sin(pipe_yaw - yaw), std::cos(pipe_yaw - yaw)));

    dlog(
        "FOLLOW: target=(%.2f,%.2f) pipe_yaw=%.1fdeg cur_yaw=%.1fdeg "
        "err=%.1fdeg",
        target_x, target_y, pipe_yaw * 180.0 / M_PI, yaw * 180.0 / M_PI,
        heading_err * 180.0 / M_PI);

    // Throttle the resend: skip if neither target XY nor pipe yaw has changed
    // enough, so we don't wipe the manager queue / reset the reference filter
    // every cycle. Force one through after max_resend_skips_ consecutive skips.
    if (have_prev_wp_) {
        const double moved =
            std::hypot(target_x - prev_wp_x_, target_y - prev_wp_y_);
        const double yaw_delta =
            std::fabs(std::atan2(std::sin(pipe_yaw - prev_wp_yaw_),
                                 std::cos(pipe_yaw - prev_wp_yaw_)));
        if (moved < min_wp_dist_ && yaw_delta < min_wp_yaw_) {
            if (++wp_skip_count_ < max_resend_skips_) {
                dlog(
                    "FOLLOW: skipped (moved=%.2f<%.2f "
                    "yaw_delta=%.1fdeg<%.1fdeg "
                    "skip %d/%d)",
                    moved, min_wp_dist_, yaw_delta * 180.0 / M_PI,
                    min_wp_yaw_ * 180.0 / M_PI, wp_skip_count_,
                    max_resend_skips_);
                return true;
            }
            wp_skip_count_ = 0;
        } else {
            wp_skip_count_ = 0;
        }
    }

    // XY_AND_YAW: move to the far pipe endpoint while holding heading along
    // the pipe direction. This keeps the drone aligned with the pipe rather
    // than angling toward the endpoint when laterally offset.
    dlog("FOLLOW: SEND XY_AND_YAW to (%.2f,%.2f) pipe_yaw=%.1fdeg", target_x,
         target_y, pipe_yaw * 180.0 / M_PI);
    auto wp = make_waypoint(target_x, target_y, pipe_yaw,
                            vortex_msgs::msg::WaypointMode::XY_AND_YAW);
    enqueue_waypoint(wp, /*overwrite_prior=*/true, /*take_priority=*/true,
                     switching_threshold_);
    prev_wp_x_ = target_x;
    prev_wp_y_ = target_y;
    prev_wp_yaw_ = pipe_yaw;
    prev_wp_mode_ = vortex_msgs::msg::WaypointMode::XY_AND_YAW;
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
    double t =
        (ab_squared == 0.0 ? 0.0 : ((ARx * ABx + ARy * ABy) / ab_squared));

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

    // Pipe direction: from A→B or B→A, whichever points toward chosen endpoint.
    double dir_x = bx - ax;
    double dir_y = by - ay;
    if (dir_x * (chosen_x - rx) + dir_y * (chosen_y - ry) < 0.0) {
        dir_x = -dir_x;
        dir_y = -dir_y;
    }
    const double pipe_yaw = std::atan2(dir_y, dir_x);

    double lookahead = get_parameter("lookahead_distance").as_double();
    double target_x = chosen_x + lookahead * std::cos(pipe_yaw);
    double target_y = chosen_y + lookahead * std::sin(pipe_yaw);

    dlog(
        "INITIAL_WP: chosen_line=%d robotBetween=%d -> endpoint=(%.2f,%.2f) "
        "lookahead=%.1fm target=(%.2f,%.2f) pipe_yaw=%.1fdeg",
        chosen_track.id, robotBetween ? 1 : 0, chosen_x, chosen_y, lookahead,
        target_x, target_y, pipe_yaw * 180.0 / M_PI);
    follow_toward(target_x, target_y, pipe_yaw);

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
    // Direction-anchored: always find the track most aligned with
    // next_line_yaw_ (or vehicle_yaw before the first junction). No ID tracking
    // needed — the directed-angle approach in get_track_by_yaw reliably rejects
    // the incoming arm (~180° from next_line_yaw_) and accepts the outgoing
    // arm.
    Track next_line;
    get_track_by_yaw(next_line);

    if (next_line.id == -1) {
        if (junction_in_progress_) {
            dlog("PUB_WP: no track (mid-turn, junction in progress) -> holding");
        } else {
            dlog("PUB_WP: no confirmed track found by yaw -> holding");
        }
        return;
    }

    // If the junction waypoint hasn't been reached yet, hold the follow
    // waypoint so the DP controller can first position the camera over the
    // corner.
    if (junction_in_progress_) {
        double dist_to_junc =
            std::hypot(orca_pose_.position.x - junction_wp_x_,
                       orca_pose_.position.y - junction_wp_y_);
        if (dist_to_junc > switching_threshold_) {
            junction_hold_ticks_++;
            dlog(
                "PUB_WP: holding follow WP — %.2fm from junction WP "
                "(%.2f,%.2f) hold_tick=%d",
                dist_to_junc, junction_wp_x_, junction_wp_y_,
                junction_hold_ticks_);
            return;
        }
        dlog("PUB_WP: reached junction WP (%.2fm) after %d ticks, releasing",
             dist_to_junc, junction_hold_ticks_);
        junction_in_progress_ = false;
        junction_hold_ticks_ = 0;
    }

    if (current_line_id_ == next_line.id) {
        current_line_id_counter_++;
    } else {
        current_line_id_ = next_line.id;
        current_line_id_counter_ = 0;
    }

    Eigen::Vector2d next_line_p1 = next_line.line_points.col(0);
    Eigen::Vector2d next_line_p2 = next_line.line_points.col(1);
    double distance1 = std::hypot(junction_wp_x_ - next_line_p1(0),
                                  junction_wp_y_ - next_line_p1(1));
    double distance2 = std::hypot(junction_wp_x_ - next_line_p2(0),
                                  junction_wp_y_ - next_line_p2(1));

    double chosen_x, chosen_y;
    if (distance1 > distance2) {
        chosen_x = next_line_p1(0);
        chosen_y = next_line_p1(1);
    } else {
        chosen_x = next_line_p2(0);
        chosen_y = next_line_p2(1);
    }

    // Pipe direction: from the near (junction-side) endpoint toward the far
    // one.
    Eigen::Vector2d near_ep =
        (distance1 > distance2) ? next_line_p2 : next_line_p1;
    const double pipe_yaw =
        std::atan2(chosen_y - near_ep(1), chosen_x - near_ep(0));

    double lookahead = get_parameter("lookahead_distance").as_double();
    double target_x = chosen_x + lookahead * std::cos(pipe_yaw);
    double target_y = chosen_y + lookahead * std::sin(pipe_yaw);

    dlog(
        "PUB_WP: next_line=%d endpoints p1=(%.2f,%.2f) p2=(%.2f,%.2f) "
        "junction=(%.2f,%.2f) -> far endpoint=(%.2f,%.2f) "
        "lookahead=%.1fm target=(%.2f,%.2f) pipe_yaw=%.1fdeg",
        next_line.id, next_line_p1(0), next_line_p1(1), next_line_p2(0),
        next_line_p2(1), junction_wp_x_, junction_wp_y_, chosen_x, chosen_y,
        lookahead, target_x, target_y, pipe_yaw * 180.0 / M_PI);

    double follow_x = target_x;
    double follow_y = target_y;

    // If the camera-corrected follow WP is already within switching_threshold
    // the camera is positioned over this track's endpoint — it's done. Find
    // the confirmed track that extends furthest from the junction in the
    // outgoing half-space (excluding the current one) and steer next_line_yaw_
    // toward it so the next tick's yaw search picks the real continuation.
    if (!used_line_intersections_.empty()) {
        const double rx = orca_pose_.position.x, ry = orca_pose_.position.y;
        const double dist_follow = std::hypot(follow_x - rx, follow_y - ry);
        if (dist_follow < switching_threshold_) {
            const double jx_r = used_line_intersections_.back().x;
            const double jy_r = used_line_intersections_.back().y;
            double best_far_dist = 0.0;
            double best_yaw = next_line_yaw_;
            for (const auto& t : line_tracker_.get_tracks()) {
                if (!t.confirmed || t.id == next_line.id)
                    continue;
                Eigen::Vector2d a = t.line_points.col(0);
                Eigen::Vector2d b = t.line_points.col(1);
                bool a_closer = (std::hypot(a(0) - jx_r, a(1) - jy_r) <
                                 std::hypot(b(0) - jx_r, b(1) - jy_r));
                Eigen::Vector2d near_ep = a_closer ? a : b;
                Eigen::Vector2d far_ep = a_closer ? b : a;
                double angle = std::atan2(far_ep(1) - near_ep(1),
                                          far_ep(0) - near_ep(0));
                double diff = std::fabs(std::atan2(
                    std::sin(angle - next_line_yaw_),
                    std::cos(angle - next_line_yaw_)));
                if (diff >= M_PI / 2.0)
                    continue;
                double far_dist =
                    std::hypot(far_ep(0) - jx_r, far_ep(1) - jy_r);
                if (far_dist > best_far_dist) {
                    best_far_dist = far_dist;
                    best_yaw = angle;
                }
            }
            dlog(
                "PUB_WP: id=%d endpoint reached (follow=%.2fm) -> "
                "next_line_yaw_ %.1f->%.1fdeg (best_far=%.2fm), releasing "
                "sticky",
                next_line.id, dist_follow, next_line_yaw_ * 180.0 / M_PI,
                best_yaw * 180.0 / M_PI, best_far_dist);
            next_line_yaw_ = best_yaw;
            current_line_id_ = -1;
            return;
        }
    }

    follow_toward(follow_x, follow_y, pipe_yaw);

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
    constexpr double kMaxYawDiff =
        M_PI / 2.0;  // 90 deg — incoming arm is ~180° away via directed angles,
                     // so this is still safe

    tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                      orca_pose_.orientation.z, orca_pose_.orientation.w);
    double roll, pitch, vehicle_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, vehicle_yaw);

    // Use next_line_yaw_ (the committed outgoing direction set at junction
    // time) as the reference when we are post-junction.  vehicle_yaw is
    // unreliable mid-turn: if the vehicle is partway between the old and new
    // pipe direction, it picks whichever pipe is nearest to its current
    // heading, causing oscillation.  next_line_yaw_ is stable and correct after
    // the directed-angle fix in set_next_line.  Before the first junction, fall
    // back to vehicle_yaw.
    const double ref_yaw =
        used_line_intersections_.empty() ? vehicle_yaw : next_line_yaw_;

    // Directed angle for a track arm: from the endpoint nearest the last
    // junction toward the far endpoint.  Every arm points away from the corner,
    // so SSA comparison against ref_yaw works without the mod-π fold.
    const double jx = used_line_intersections_.empty()
                          ? orca_pose_.position.x
                          : used_line_intersections_.back().x;
    const double jy = used_line_intersections_.empty()
                          ? orca_pose_.position.y
                          : used_line_intersections_.back().y;
    auto directed_angle = [&](const Track& t) -> double {
        Eigen::Vector2d a = t.line_points.col(0);
        Eigen::Vector2d b = t.line_points.col(1);
        bool a_closer = (std::hypot(a(0) - jx, a(1) - jy) <
                         std::hypot(b(0) - jx, b(1) - jy));
        Eigen::Vector2d near = a_closer ? a : b;
        Eigen::Vector2d far = a_closer ? b : a;
        return std::atan2(far(1) - near(1), far(0) - near(0));
    };

    // Sticky: once we have committed to a confirmed track, keep following it
    // rather than re-evaluating every tick. This prevents spurious switches when
    // a secondary track (e.g. a visual artefact near the pipe endpoint) appears
    // with an angle closer to next_line_yaw_ than the real pipe. The lock is
    // cleared when a junction fires (current_line_id_ reset to -1 in
    // publish_intersection), so the outgoing arm is still selected by yaw.
    if (current_line_id_ != -1 && !junction_in_progress_) {
        Track sticky{};
        if (get_track_by_id(sticky, current_line_id_) != -1 && sticky.confirmed) {
            double angle = directed_angle(sticky);
            // Keep next_line_yaw_ in sync with the actual pipe direction so
            // the next junction's set_next_line() uses an accurate ref_yaw.
            next_line_yaw_ = angle;
            dlog("GET_TRACK_YAW: sticky id=%d ang=%.1fdeg", sticky.id,
                 angle * 180.0 / M_PI);
            line_track = sticky;
            return;
        }
        dlog("GET_TRACK_YAW: sticky id=%d gone/unconfirmed -> yaw search",
             current_line_id_);
    }

    auto best_by_yaw = [&](bool confirmed_only) -> std::pair<double, Track> {
        double min_diff = std::numeric_limits<double>::max();
        Track best{};
        for (const auto& track : line_tracker_.get_tracks()) {
            if (confirmed_only && !track.confirmed)
                continue;
            if (!confirmed_only && track.confirmed)
                continue;
            double angle = directed_angle(track);
            double diff = std::fabs(std::atan2(std::sin(angle - ref_yaw),
                                               std::cos(angle - ref_yaw)));
            if (diff < min_diff) {
                min_diff = diff;
                best = track;
            }
        }
        return {min_diff, best};
    };

    // First pass: confirmed tracks (prevents picking the incoming line, which
    // stays confirmed for several cycles after the junction).
    auto [cdiff, cbest] = best_by_yaw(true);
    if (cdiff <= kMaxYawDiff) {
        dlog("GET_TRACK_YAW: confirmed id=%d diff=%.1fdeg ref_yaw=%.1fdeg",
             cbest.id, cdiff * 180.0 / M_PI, ref_yaw * 180.0 / M_PI);
        line_track = cbest;
        return;
    }

    // Second pass: unconfirmed tracks, still gated by the same angle threshold.
    // After a corner the outgoing line track is often deleted during the turn
    // and its replacement takes a few cycles to confirm; allowing unconfirmed
    // here prevents the multi-second gap in waypoint output while it
    // reconverges.
    auto [udiff, ubest] = best_by_yaw(false);
    if (udiff <= kMaxYawDiff) {
        dlog(
            "GET_TRACK_YAW: no confirmed match; using unconfirmed id=%d "
            "diff=%.1fdeg",
            ubest.id, udiff * 180.0 / M_PI);
        line_track = ubest;
        return;
    }

    dlog(
        "GET_TRACK_YAW: best match diff=%.1f deg > threshold %.1f deg "
        "(ref_yaw=%.1fdeg) -> hold",
        std::min(cdiff, udiff) * 180.0 / M_PI, kMaxYawDiff * 180.0 / M_PI,
        ref_yaw * 180.0 / M_PI);
    line_track = Track{};
}

void LineFilteringNode::termination_check() {
    const auto& ltracks = line_tracker_.get_tracks();

    if (termination_track_count_pub_) {
        std_msgs::msg::Int32 count_msg;
        count_msg.data = static_cast<int32_t>(ltracks.size());
        termination_track_count_pub_->publish(count_msg);
    }

    Track line_track;
    if (get_track_by_id(line_track, current_line_id_) == -1) {
        dlog(
            "TERMINATION: lost track of cur_line=%d (line_tracks=%zu) -- "
            "counter reset",
            current_line_id_, ltracks.size());
        termination_counter_ = 0;
        return;
    }
    Eigen::Vector2d line_p1 = line_track.line_points.col(0);
    Eigen::Vector2d line_p2 = line_track.line_points.col(1);
    auto intersection = used_line_intersections_.back();
    Eigen::Vector2d orca_position(orca_pose_.position.x, orca_pose_.position.y);
    double distance1 =
        std::hypot(intersection.x - line_p1(0), intersection.y - line_p1(1));
    double distance2 =
        std::hypot(intersection.x - line_p2(0), intersection.y - line_p2(1));
    Eigen::Vector2d endpoint = (distance1 > distance2) ? line_p1 : line_p2;
    double distance = std::hypot(orca_position(0) - endpoint(0),
                                 orca_position(1) - endpoint(1));
    double dist_from_junction = std::hypot(orca_position(0) - intersection.x,
                                           orca_position(1) - intersection.y);
    if (distance < 0.5 && dist_from_junction > 1.0) {
        termination_counter_++;
    } else {
        termination_counter_ = 0;
    }

    const int threshold =
        this->get_parameter("termination_counter_threshold").as_int();
    dlog(
        "TERMINATION: cur_line=%d endpoint=(%.2f,%.2f) orca=(%.2f,%.2f) "
        "dist_to_endpoint=%.2f counter=%d/%d line_tracks=%zu",
        current_line_id_, endpoint(0), endpoint(1), orca_position(0),
        orca_position(1), distance, termination_counter_, threshold,
        ltracks.size());

    if (termination_counter_ > threshold) {
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
