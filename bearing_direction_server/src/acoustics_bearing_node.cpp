#include "bearing_direction_server/acoustics_bearing_node.hpp"

#include <spdlog/spdlog.h>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace bearing_direction_server {

AcousticsBearingNode::AcousticsBearingNode(const rclcpp::NodeOptions& options)
    : BearingDirectionBase("acoustics_bearing_server", options) {
    declare_parameter<std::string>("topics.bearing_measurements",
                                   "acoustics/bearing_measurements");
    declare_parameter<std::string>("frame_override", "");

    bearing_sub_ = create_subscription<vortex_msgs::msg::BearingMeasurement>(
        get_parameter("topics.bearing_measurements").as_string(),
        vortex::utils::qos_profiles::sensor_data_profile(1),
        std::bind(&AcousticsBearingNode::bearing_callback, this,
                  std::placeholders::_1));

    setup_base();
}

void AcousticsBearingNode::bearing_callback(
    const vortex_msgs::msg::BearingMeasurement::SharedPtr msg) {
    {
        std::lock_guard lock(mutex_);
        if (!collecting_)
            return;
    }

    const auto& bearing = msg->bearing;
    const Eigen::Vector3d dir(bearing.vector.x, bearing.vector.y,
                              bearing.vector.z);
    const double norm = dir.norm();
    if (!std::isfinite(norm) || norm < 1e-6)
        return;

    const std::string override_frame =
        get_parameter("frame_override").as_string();
    const std::string src_frame =
        override_frame.empty() ? bearing.header.frame_id : override_frame;
    if (src_frame.empty())
        return;

    const auto dir_odom = rotate_to_odom(dir / norm, src_frame,
                                         rclcpp::Time(bearing.header.stamp));
    if (!dir_odom) {
        spdlog::warn(
            "AcousticsBearingNode: TF lookup failed, dropping measurement.");
        return;
    }

    const double new_weight = msg->weight > 1e-9 ? msg->weight : 1.0;

    Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
    {
        std::lock_guard lock(mutex_);
        if (latest_drone_pos_)
            odom_pos =
                Eigen::Vector3d(latest_drone_pos_->x, latest_drone_pos_->y,
                                latest_drone_pos_->z);
    }

    // Running average filter — mirrors the Python reference implementation.
    // Blends both direction and position, accumulating confidence via
    // combined_weight.
    const double combined_weight = filter_.weight + new_weight;
    const double safe_weight =
        combined_weight > 1e-12 ? combined_weight : 1e-12;

    const Eigen::Vector3d blended_pos =
        (filter_.pos * filter_.weight + odom_pos * new_weight) / safe_weight;

    Eigen::Vector3d blended_dir =
        filter_.dir * filter_.weight + *dir_odom * new_weight;
    const double blended_norm = blended_dir.norm();
    if (blended_norm > 1e-9) {
        blended_dir /= blended_norm;
    } else {
        // Fallback: keep whichever direction has more confidence
        const double prev_norm = filter_.dir.norm();
        const double new_norm = dir_odom->norm();
        if (new_norm > prev_norm && new_norm > 1e-9)
            blended_dir = *dir_odom / new_norm;
        else if (prev_norm > 1e-9)
            blended_dir = filter_.dir / prev_norm;
        else
            blended_dir = *dir_odom;
    }

    filter_ = FilterState{blended_dir, blended_pos, combined_weight};

    spdlog::info(
        "AcousticsBearingNode: weight={:.2f} dir=[{:.3f},{:.3f},{:.3f}] "
        "pos=[{:.2f},{:.2f},{:.2f}]",
        combined_weight, blended_dir.x(), blended_dir.y(), blended_dir.z(),
        blended_pos.x(), blended_pos.y(), blended_pos.z());

    // Call add_direction with raw values so the base class tracks measurement
    // count correctly (min/max_measurements, feedback).
    // The final result uses filter_.dir and filter_.pos via
    // final_result_override_ so intermediate running-average states don't bias
    // the waypoint.
    add_direction(*dir_odom, odom_pos);

    {
        std::lock_guard lock(mutex_);
        final_result_override_ = std::make_pair(blended_dir, blended_pos);
    }
}

}  // namespace bearing_direction_server
