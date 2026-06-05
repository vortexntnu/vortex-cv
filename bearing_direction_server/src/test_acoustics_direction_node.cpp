#include "bearing_direction_server/test_acoustics_direction_node.hpp"

#include <spdlog/spdlog.h>
#include <cmath>

namespace bearing_direction_server {

TestAcousticsDirectionNode::TestAcousticsDirectionNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("test_acoustics_direction_node", options),
      rng_(std::random_device{}()) {
    declare_parameter<double>("target.x", 1.777425535555405);
    declare_parameter<double>("target.y", -0.053997583427242916);
    declare_parameter<double>("target.z", 5.301166999942316);
    declare_parameter<double>("noise_std_deg", 3.0);
    declare_parameter<std::string>("topics.odom", "/nautilus/odom");
    declare_parameter<std::string>("topics.bearing_measurements",
                                   "acoustics/bearing_measurements");
    declare_parameter<double>("publish_rate_hz", 10.0);

    target_pos_ = {
        get_parameter("target.x").as_double(),
        get_parameter("target.y").as_double(),
        get_parameter("target.z").as_double(),
    };

    noise_std_deg_ = get_parameter("noise_std_deg").as_double();
    const double noise_std_rad = noise_std_deg_ * M_PI / 180.0;
    angle_dist_ = std::normal_distribution<double>(0.0, noise_std_rad);
    azimuth_dist_ = std::uniform_real_distribution<double>(0.0, 2.0 * M_PI);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("topics.odom").as_string(), 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            odom_callback(msg);
        });

    bearing_pub_ =
        create_publisher<vortex_msgs::msg::BearingMeasurementArray>(
            get_parameter("topics.bearing_measurements").as_string(), 10);

    const double period_s =
        1.0 / get_parameter("publish_rate_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(period_s),
                               [this]() { publish_bearing(); });

    spdlog::info(
        "TestAcousticsDirectionNode ready. "
        "target=[{:.3f},{:.3f},{:.3f}], noise_std={:.1f}deg",
        target_pos_.x(), target_pos_.y(), target_pos_.z(), noise_std_deg_);
}

void TestAcousticsDirectionNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard lock(mutex_);
    drone_pos_ = {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z,
    };
}

Eigen::Vector3d TestAcousticsDirectionNode::add_angular_noise(
    const Eigen::Vector3d& dir) {
    const double theta = angle_dist_(rng_);
    const double phi = azimuth_dist_(rng_);

    Eigen::Vector3d perp;
    if (std::abs(dir.x()) < 0.9) {
        perp = dir.cross(Eigen::Vector3d::UnitX()).normalized();
    } else {
        perp = dir.cross(Eigen::Vector3d::UnitY()).normalized();
    }
    const Eigen::Vector3d perp2 = dir.cross(perp).normalized();
    const Eigen::Vector3d noise_axis =
        std::cos(phi) * perp + std::sin(phi) * perp2;

    return (std::cos(theta) * dir + std::sin(theta) * noise_axis).normalized();
}

void TestAcousticsDirectionNode::publish_bearing() {
    std::optional<Eigen::Vector3d> drone_pos;
    {
        std::lock_guard lock(mutex_);
        drone_pos = drone_pos_;
    }

    if (!drone_pos) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "No odom yet — waiting for %s",
                             get_parameter("topics.odom").as_string().c_str());
        return;
    }

    const Eigen::Vector3d diff = target_pos_ - *drone_pos;
    const double dist = diff.norm();
    if (dist < 1e-3) {
        RCLCPP_WARN(get_logger(), "Drone is at target position, skipping.");
        return;
    }

    const Eigen::Vector3d dir_noisy = add_angular_noise(diff / dist);

    const auto stamp = now();

    vortex_msgs::msg::BearingMeasurementArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "nautilus/odom";

    vortex_msgs::msg::BearingMeasurement meas;
    meas.bearing.header.stamp = stamp;
    meas.bearing.header.frame_id = "nautilus/odom";
    meas.bearing.vector.x = dir_noisy.x();
    meas.bearing.vector.y = dir_noisy.y();
    meas.bearing.vector.z = dir_noisy.z();
    meas.weight = 1.0;
    meas.target_id = 0;

    msg.bearings.push_back(meas);
    bearing_pub_->publish(msg);
}

}  // namespace bearing_direction_server
