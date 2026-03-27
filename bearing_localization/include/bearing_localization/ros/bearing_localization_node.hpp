#ifndef BEARING_LOCALIZATION__ROS__BEARING_LOCALIZATION_NODE_HPP_
#define BEARING_LOCALIZATION__ROS__BEARING_LOCALIZATION_NODE_HPP_

#include "bearing_localization/lib/bearing_localization_config.hpp"
#include "bearing_localization/lib/bearing_localizer.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/vector3_array.hpp>

#include <memory>
#include <string>

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

    void publish_result(const LocalizationResult& result);

    BearingLocalizationConfig cfg_;
    std::unique_ptr<BearingLocalizer> localizer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        bearing_sub_;
    rclcpp::Subscription<vortex_msgs::msg::Vector3Array>::SharedPtr
        bearing_array_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__ROS__BEARING_LOCALIZATION_NODE_HPP_
