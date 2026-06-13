#ifndef BEARING_DIRECTION_SERVER__TEST_ACOUSTICS_DIRECTION_NODE_HPP_
#define BEARING_DIRECTION_SERVER__TEST_ACOUSTICS_DIRECTION_NODE_HPP_

#include <mutex>
#include <optional>
#include <random>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/bearing_measurement.hpp>
#include <vortex_msgs/msg/bearing_measurement_array.hpp>

namespace bearing_direction_server {

/**
 * Test publisher that computes a direction vector from the drone (via odom)
 * to a fixed target point, adds Gaussian angular noise, and publishes it
 * as a BearingMeasurementArray so the acoustics_bearing_server can consume it.
 *
 * The direction is published already in the odom frame so no TF lookup is
 * needed on the server side.
 */
class TestAcousticsDirectionNode : public rclcpp::Node {
   public:
    explicit TestAcousticsDirectionNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_bearing();
    Eigen::Vector3d add_angular_noise(const Eigen::Vector3d& dir);

    Eigen::Vector3d target_pos_;
    std::optional<Eigen::Vector3d> drone_pos_;
    double noise_std_deg_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<vortex_msgs::msg::BearingMeasurement>::SharedPtr
        bearing_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    std::mt19937 rng_;
    std::normal_distribution<double> angle_dist_;
    std::uniform_real_distribution<double> azimuth_dist_;
};

}  // namespace bearing_direction_server

#endif  // BEARING_DIRECTION_SERVER__TEST_ACOUSTICS_DIRECTION_NODE_HPP_
