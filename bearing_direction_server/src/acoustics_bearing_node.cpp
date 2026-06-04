#include "bearing_direction_server/acoustics_bearing_node.hpp"

#include <spdlog/spdlog.h>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace bearing_direction_server {

AcousticsBearingNode::AcousticsBearingNode(const rclcpp::NodeOptions& options)
    : BearingDirectionBase("acoustics_bearing_server", options) {
    declare_parameter<std::string>("topics.bearing_measurements",
                                   "acoustics/bearing_measurements");

    bearing_sub_ =
        create_subscription<vortex_msgs::msg::BearingMeasurementArray>(
            get_parameter("topics.bearing_measurements").as_string(),
            vortex::utils::qos_profiles::sensor_data_profile(1),
            std::bind(&AcousticsBearingNode::bearing_callback, this,
                      std::placeholders::_1));

    setup_base();
}

void AcousticsBearingNode::bearing_callback(
    const vortex_msgs::msg::BearingMeasurementArray::SharedPtr msg) {
    {
        std::lock_guard lock(mutex_);
        if (!collecting_) return;
    }

    for (const auto& measurement : msg->bearings) {
        const auto& bearing = measurement.bearing;
        const Eigen::Vector3d dir(bearing.vector.x, bearing.vector.y,
                                  bearing.vector.z);
        const double norm = dir.norm();
        if (!std::isfinite(norm) || norm < 1e-6) continue;

        const std::string& src_frame = bearing.header.frame_id;
        if (src_frame.empty()) continue;

        const auto dir_odom = rotate_to_odom(
            dir / norm, src_frame, rclcpp::Time(bearing.header.stamp));
        if (dir_odom) add_direction(*dir_odom);
    }
}

}  // namespace bearing_direction_server
