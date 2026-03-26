#pragma once

#include "bearing_localization/geometry_checks.hpp"
#include "bearing_localization/measurement_buffer.hpp"
#include "bearing_localization/ray_measurement.hpp"
#include "bearing_localization/triangulation_solver.hpp"
#include "vortex/utils/ros/qos_profiles.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/vector3_array.hpp>

#include <string>
#include <vector>

#include <memory>

namespace bearing_localization {

class BearingLocalizationNode : public rclcpp::Node {
   public:
    explicit BearingLocalizationNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void declare_parameters();
    void setup_pubsub();

    void bearing_callback(
        const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void bearing_array_callback(
        const vortex_msgs::msg::Vector3Array::SharedPtr msg);

    bool process_bearing(const geometry_msgs::msg::Vector3& vec,
                         const std_msgs::msg::Header& header);
    void try_solve_and_publish();

    void publish_result(const Eigen::Vector3d& position, double residual);
    void publish_debug_markers(const std::vector<RayMeasurement>& rays,
                               const Eigen::Vector3d& position);

    std::string target_frame_;
    int window_size_;
    double max_measurement_age_sec_;
    int min_measurements_;
    double min_baseline_m_;
    double min_ray_angle_deg_;
    double outlier_residual_threshold_m_;
    int max_outlier_iterations_;
    bool publish_markers_;
    int landmark_type_;
    int landmark_subtype_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<MeasurementBuffer> buffer_;
    TriangulationSolver solver_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        bearing_sub_;
    rclcpp::Subscription<vortex_msgs::msg::Vector3Array>::SharedPtr
        bearing_array_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;
};

}  // namespace bearing_localization
