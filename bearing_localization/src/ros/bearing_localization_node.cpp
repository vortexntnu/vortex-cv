#include "bearing_localization/ros/bearing_localization_node.hpp"
#include "bearing_localization/ros/debug_markers.hpp"

#include <spdlog/spdlog.h>
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

    node_cfg_.target_frame = get_parameter("target_frame").as_string();
    node_cfg_.publish_debug_markers =
        get_parameter("publish_debug_markers").as_bool();
    node_cfg_.landmark_type = get_parameter("landmark_type").as_int();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    setup_pubsub();

    spdlog::info(
        "BearingLocalizationNode started. config_file='{}', "
        "target_frame='{}'",
        config_file, node_cfg_.target_frame);
}

void BearingLocalizationNode::declare_parameters() {
    declare_parameter<std::string>("config_file");
    declare_parameter<std::string>("target_frame");
    declare_parameter<bool>("publish_debug_markers");
    declare_parameter<int>("landmark_type");

    declare_parameter<std::string>("topics.bearing_measurements");
    declare_parameter<std::string>("topics.landmarks");
    declare_parameter<std::string>("topics.bearing_localization_markers");
}

void BearingLocalizationNode::setup_pubsub() {
    bearing_sub_ =
        create_subscription<vortex_msgs::msg::BearingMeasurementArray>(
            get_parameter("topics.bearing_measurements").as_string(),
            vortex::utils::qos_profiles::sensor_data_profile(1),
            std::bind(&BearingLocalizationNode::bearing_callback, this,
                      std::placeholders::_1));

    if (node_cfg_.publish_debug_markers) {
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            get_parameter("topics.bearing_localization_markers").as_string(),
            vortex::utils::qos_profiles::reliable_profile());
    }

    landmark_pub_ = create_publisher<vortex_msgs::msg::LandmarkArray>(
        get_parameter("topics.landmarks").as_string(),
        vortex::utils::qos_profiles::reliable_profile());
}

void BearingLocalizationNode::bearing_callback(
    const vortex_msgs::msg::BearingMeasurementArray::SharedPtr msg) {
    std::unordered_map<int32_t, bool> updated_targets;

    for (const auto& measurement : msg->bearings) {
        const auto& bearing = measurement.bearing;
        auto ray = transform_bearing(bearing.vector, bearing.header,
                                     measurement.weight);
        if (!ray) {
            continue;
        }

        const int32_t target_id = measurement.target_id;
        get_localizer(target_id).add_measurement(*ray);
        updated_targets[target_id] = true;
    }

    for (const auto& [target_id, _] : updated_targets) {
        auto result = get_localizer(target_id).solve(now().seconds());
        if (!result) {
            continue;
        }
        publish_result(*result, target_id);
        if (node_cfg_.publish_debug_markers && markers_pub_) {
            markers_pub_->publish(
                build_debug_markers(*result, node_cfg_.target_frame, now()));
        }
    }
}

std::optional<RayMeasurement> BearingLocalizationNode::transform_bearing(
    const geometry_msgs::msg::Vector3& vec,
    const std_msgs::msg::Header& header,
    double weight) {
    Eigen::Vector3d dir(vec.x, vec.y, vec.z);
    const double norm = dir.norm();
    if (!std::isfinite(norm) || norm < 1e-6) {
        spdlog::warn("Invalid direction vector (norm={:.6f}), skipping.", norm);
        return std::nullopt;
    }
    dir /= norm;

    const std::string& source_frame = header.frame_id;
    if (source_frame.empty()) {
        spdlog::warn("Received measurement with empty frame_id, skipping.");
        return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            node_cfg_.target_frame, source_frame, rclcpp::Time(header.stamp));
    } catch (const tf2::ExtrapolationException&) {
        try {
            tf_stamped = tf_buffer_->lookupTransform(
                node_cfg_.target_frame, source_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            spdlog::warn("TF lookup failed ({} -> {}): {}", source_frame,
                         node_cfg_.target_frame, ex.what());
            return std::nullopt;
        }
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("TF lookup failed ({} -> {}): {}", source_frame,
                     node_cfg_.target_frame, ex.what());
        return std::nullopt;
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
    ray.weight = weight;
    return ray;
}

BearingLocalizer& BearingLocalizationNode::get_localizer(int32_t target_id) {
    auto it = target_localizers_.find(target_id);
    if (it != target_localizers_.end()) {
        return *it->second;
    }
    auto [inserted, _] = target_localizers_.emplace(
        target_id, std::make_unique<BearingLocalizer>(cfg_));
    return *inserted->second;
}

void BearingLocalizationNode::publish_result(const LocalizationResult& result,
                                             int32_t target_id) {
    auto stamp = now();

    vortex_msgs::msg::Landmark landmark;
    landmark.header.stamp = stamp;
    landmark.header.frame_id = node_cfg_.target_frame;
    landmark.id = 0;
    landmark.type.value = static_cast<uint16_t>(node_cfg_.landmark_type);
    landmark.subtype.value = static_cast<uint16_t>(target_id);
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
    landmark_array.header.frame_id = node_cfg_.target_frame;
    landmark_array.landmarks.push_back(landmark);
    landmark_pub_->publish(landmark_array);
}

}  // namespace bearing_localization
