#include "bearing_localization/bearing_localization_node.hpp"

#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace bearing_localization {

BearingLocalizationNode::BearingLocalizationNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("bearing_localization_node", options) {
    declare_parameters();

    const std::string profile = get_parameter("profile").as_string();
    auto p = [&](const std::string& name) {
        const std::string profiled = profile + "." + name;
        return has_parameter(profiled) ? profiled : name;
    };

    target_frame_ = get_parameter(p("target_frame")).as_string();
    window_size_ = get_parameter(p("window_size")).as_int();
    max_measurement_age_sec_ =
        get_parameter(p("max_measurement_age_sec")).as_double();
    min_measurements_ = get_parameter(p("min_measurements")).as_int();
    min_baseline_m_ = get_parameter(p("min_baseline_m")).as_double();
    min_ray_angle_deg_ = get_parameter(p("min_ray_angle_deg")).as_double();
    outlier_residual_threshold_m_ =
        get_parameter(p("outlier_residual_threshold_m")).as_double();
    max_outlier_iterations_ =
        get_parameter(p("max_outlier_iterations")).as_int();
    publish_markers_ = get_parameter(p("publish_markers")).as_bool();
    landmark_type_ = get_parameter(p("landmark_type")).as_int();
    landmark_subtype_ = get_parameter(p("landmark_subtype")).as_int();
    landmark_id_ = get_parameter(p("landmark_id")).as_int();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    buffer_ = std::make_unique<MeasurementBuffer>(
        static_cast<size_t>(window_size_), max_measurement_age_sec_);

    setup_pubsub();

    RCLCPP_INFO(get_logger(),
                "BearingLocalizationNode started. target_frame='%s'",
                target_frame_.c_str());
}

void BearingLocalizationNode::declare_parameters() {
    declare_parameter("profile", "default");
    declare_parameter("target_frame", "orca/odom");
    declare_parameter("window_size", 30);
    declare_parameter("max_measurement_age_sec", 5.0);
    declare_parameter("min_measurements", 5);
    declare_parameter("min_baseline_m", 0.5);
    declare_parameter("min_ray_angle_deg", 5.0);
    declare_parameter("outlier_residual_threshold_m", 1.0);
    declare_parameter("max_outlier_iterations", 2);
    declare_parameter("publish_markers", true);
    declare_parameter("landmark_type", 0);
    declare_parameter("landmark_subtype", 0);
    declare_parameter("landmark_id", 0);

    declare_parameter("sub_topics.bearing", "/bearing_measurement");
    declare_parameter("sub_topics.bearing_array",
                      "/aruco_detector/marker_directions");
    declare_parameter("pub_topics.landmarks", "/orca/landmarks");
    declare_parameter("pub_topics.markers", "/bearing_localization/markers");
}

void BearingLocalizationNode::setup_pubsub() {
    bearing_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        get_parameter("sub_topics.bearing").as_string(), 10,
        std::bind(&BearingLocalizationNode::bearing_callback, this,
                  std::placeholders::_1));

    auto qos_sensor = rclcpp::QoS(
        rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 1),
        rmw_qos_profile_sensor_data);

    bearing_array_sub_ = create_subscription<vortex_msgs::msg::Vector3Array>(
        get_parameter("sub_topics.bearing_array").as_string(), qos_sensor,
        std::bind(&BearingLocalizationNode::bearing_array_callback, this,
                  std::placeholders::_1));

    if (publish_markers_) {
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            get_parameter("pub_topics.markers").as_string(), 10);
    }

    landmark_pub_ = create_publisher<vortex_msgs::msg::LandmarkArray>(
        get_parameter("pub_topics.landmarks").as_string(), 10);
}

void BearingLocalizationNode::bearing_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    process_bearing(msg->vector, msg->header);
    try_solve_and_publish();
}

void BearingLocalizationNode::bearing_array_callback(
    const vortex_msgs::msg::Vector3Array::SharedPtr msg) {
    for (const auto& vec : msg->vectors) {
        process_bearing(vec, msg->header);
    }
    try_solve_and_publish();
}

bool BearingLocalizationNode::process_bearing(
    const geometry_msgs::msg::Vector3& vec,
    const std_msgs::msg::Header& header) {
    Eigen::Vector3d dir(vec.x, vec.y, vec.z);
    const double norm = dir.norm();
    if (!std::isfinite(norm) || norm < 1e-6) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Invalid direction vector (norm=%.6f), skipping.",
                             norm);
        return false;
    }
    dir /= norm;

    // TF transform lookup
    const std::string& source_frame = header.frame_id;
    if (source_frame.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Received measurement with empty frame_id, skipping.");
        return false;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(target_frame_, source_frame,
                                                 tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "TF lookup failed (%s -> %s): %s",
            source_frame.c_str(), target_frame_.c_str(), ex.what());
        return false;
    }

    const auto& rot = tf_stamped.transform.rotation;
    const Eigen::Quaterniond q(rot.w, rot.x, rot.y, rot.z);
    const Eigen::Vector3d dir_world = (q * dir).normalized();

    const auto& tr = tf_stamped.transform.translation;
    const Eigen::Vector3d origin(tr.x, tr.y, tr.z);

    RayMeasurement ray;
    ray.stamp = header.stamp;
    ray.origin_world = origin;
    ray.direction_world = dir_world;
    buffer_->add(ray);
    return true;
}

void BearingLocalizationNode::try_solve_and_publish() {
    buffer_->prune(now());
    const auto& rays = buffer_->rays();

    if (static_cast<int>(rays.size()) < min_measurements_) {
        return;
    }

    const auto pre_check = GeometryChecks::check_pre_solve(
        rays, static_cast<size_t>(min_measurements_), min_baseline_m_,
        min_ray_angle_deg_);
    if (!pre_check.passed) {
        RCLCPP_DEBUG(get_logger(), "Pre-solve check failed: %s",
                     pre_check.reason.c_str());
        return;
    }

    const SolverResult result = solver_.solve_with_outlier_rejection(
        rays, outlier_residual_threshold_m_, max_outlier_iterations_);

    if (!result.valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Solver returned invalid result.");
        return;
    }

    std::vector<RayMeasurement> inlier_rays;
    inlier_rays.reserve(result.inlier_indices.size());
    for (size_t idx : result.inlier_indices) {
        inlier_rays.push_back(rays[idx]);
    }

    const auto depth_check =
        GeometryChecks::check_post_solve(inlier_rays, result.position);
    if (!depth_check.passed) {
        RCLCPP_DEBUG(get_logger(), "Depth check failed: %s",
                     depth_check.reason.c_str());
        return;
    }

    publish_result(result.position, result.mean_residual);

    if (publish_markers_ && markers_pub_) {
        publish_debug_markers(rays, result.position);
    }
}

void BearingLocalizationNode::publish_result(const Eigen::Vector3d& position,
                                             double residual) {
    auto stamp = now();

    vortex_msgs::msg::Landmark landmark;
    landmark.header.stamp = stamp;
    landmark.header.frame_id = target_frame_;
    landmark.id = landmark_id_;
    landmark.type.value = static_cast<uint16_t>(landmark_type_);
    landmark.subtype.value = static_cast<uint16_t>(landmark_subtype_);
    landmark.pose.pose.position.x = position.x();
    landmark.pose.pose.position.y = position.y();
    landmark.pose.pose.position.z = position.z();
    landmark.pose.pose.orientation.w = 1.0;
    const double var = residual * residual;
    landmark.pose.covariance[0] = var;
    landmark.pose.covariance[7] = var;
    landmark.pose.covariance[14] = var;

    vortex_msgs::msg::LandmarkArray landmark_array;
    landmark_array.header.stamp = stamp;
    landmark_array.header.frame_id = target_frame_;
    landmark_array.landmarks.push_back(landmark);
    landmark_pub_->publish(landmark_array);
}

void BearingLocalizationNode::publish_debug_markers(
    const std::vector<RayMeasurement>& rays,
    const Eigen::Vector3d& position) {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(del);

    int id = 0;
    for (const auto& ray : rays) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = now();
        arrow.header.frame_id = target_frame_;
        arrow.ns = "bearing_rays";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.05;  // shaft diameter
        arrow.scale.y = 0.10;  // head diameter
        arrow.scale.z = 0.10;  // head length
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

    // Point at estimated target position
    visualization_msgs::msg::Marker sphere;
    sphere.header.stamp = now();
    sphere.header.frame_id = target_frame_;
    sphere.ns = "bearing_target";
    sphere.id = 0;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = position.x();
    sphere.pose.position.y = position.y();
    sphere.pose.position.z = position.z();
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

    markers_pub_->publish(array);
}

}  // namespace bearing_localization
