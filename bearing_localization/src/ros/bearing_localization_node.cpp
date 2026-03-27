#include "bearing_localization/ros/bearing_localization_node.hpp"
#include "bearing_localization/ros/debug_markers.hpp"

#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "vortex/utils/ros/qos_profiles.hpp"

namespace bearing_localization {

BearingLocalizationNode::BearingLocalizationNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("bearing_localization_node", options) {
    declare_parameters();

    const std::string config_file = get_parameter("config_file").as_string();
    cfg_ = BearingLocalizationConfig::from_yaml(config_file);

    localizer_ = std::make_unique<BearingLocalizer>(cfg_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    setup_pubsub();

    RCLCPP_INFO(get_logger(),
                "BearingLocalizationNode started. config_file='%s', "
                "target_frame='%s'",
                config_file.c_str(), cfg_.target_frame.c_str());
}

void BearingLocalizationNode::declare_parameters() {
    declare_parameter<std::string>("config_file");

    declare_parameter<std::string>("topics.bearing_measurement");
    declare_parameter<std::string>("topics.bearing_array");
    declare_parameter<std::string>("topics.landmarks");
    declare_parameter<std::string>("topics.bearing_localization_markers");
}

void BearingLocalizationNode::setup_pubsub() {
    bearing_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        get_parameter("topics.bearing_measurement").as_string(),
        vortex::utils::qos_profiles::sensor_data_profile(1),
        std::bind(&BearingLocalizationNode::bearing_callback, this,
                  std::placeholders::_1));

    bearing_array_sub_ = create_subscription<vortex_msgs::msg::Vector3Array>(
        get_parameter("topics.bearing_array").as_string(),
        vortex::utils::qos_profiles::sensor_data_profile(1),
        std::bind(&BearingLocalizationNode::bearing_array_callback, this,
                  std::placeholders::_1));

    if (cfg_.publish_debug_markers) {
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            get_parameter("topics.bearing_localization_markers").as_string(),
            vortex::utils::qos_profiles::reliable_profile());
    }

    landmark_pub_ = create_publisher<vortex_msgs::msg::LandmarkArray>(
        get_parameter("topics.landmarks").as_string(),
        vortex::utils::qos_profiles::reliable_profile());
}

void BearingLocalizationNode::bearing_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    process_bearing(msg->vector, msg->header);

    auto result = localizer_->solve(now().seconds());
    if (!result) {
        return;
    }
    publish_result(*result);
    if (cfg_.publish_debug_markers && markers_pub_) {
        markers_pub_->publish(
            build_debug_markers(*result, cfg_.target_frame, now()));
    }
}

void BearingLocalizationNode::bearing_array_callback(
    const vortex_msgs::msg::Vector3Array::SharedPtr msg) {
    for (const auto& vec : msg->vectors) {
        process_bearing(vec, msg->header);
    }

    auto result = localizer_->solve(now().seconds());
    if (!result) {
        return;
    }
    publish_result(*result);
    if (cfg_.publish_debug_markers && markers_pub_) {
        markers_pub_->publish(
            build_debug_markers(*result, cfg_.target_frame, now()));
    }
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

    const std::string& source_frame = header.frame_id;
    if (source_frame.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Received measurement with empty frame_id, skipping.");
        return false;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            cfg_.target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "TF lookup failed (%s -> %s): %s",
            source_frame.c_str(), cfg_.target_frame.c_str(), ex.what());
        return false;
    }

    const auto& rot = tf_stamped.transform.rotation;
    const Eigen::Quaterniond q(rot.w, rot.x, rot.y, rot.z);
    const Eigen::Vector3d dir_world = (q * dir).normalized();

    const auto& tr = tf_stamped.transform.translation;
    const Eigen::Vector3d origin(tr.x, tr.y, tr.z);

    RayMeasurement ray;
    ray.stamp_sec = rclcpp::Time(header.stamp).seconds();
    ray.origin_world = origin;
    ray.direction_world = dir_world;
    localizer_->add_measurement(ray);
    return true;
}

void BearingLocalizationNode::publish_result(const LocalizationResult& result) {
    auto stamp = now();

    vortex_msgs::msg::Landmark landmark;
    landmark.header.stamp = stamp;
    landmark.header.frame_id = cfg_.target_frame;
    landmark.id = 0;
    landmark.type.value = static_cast<uint16_t>(cfg_.landmark_type);
    landmark.subtype.value = static_cast<uint16_t>(cfg_.landmark_subtype);
    landmark.pose.pose.position.x = result.position.x();
    landmark.pose.pose.position.y = result.position.y();
    landmark.pose.pose.position.z = result.position.z();
    landmark.pose.pose.orientation.w = 1.0;
    const double var = result.mean_residual * result.mean_residual;
    landmark.pose.covariance[0] = var;
    landmark.pose.covariance[7] = var;
    landmark.pose.covariance[14] = var;

    vortex_msgs::msg::LandmarkArray landmark_array;
    landmark_array.header.stamp = stamp;
    landmark_array.header.frame_id = cfg_.target_frame;
    landmark_array.landmarks.push_back(landmark);
    landmark_pub_->publish(landmark_array);
}

}  // namespace bearing_localization
