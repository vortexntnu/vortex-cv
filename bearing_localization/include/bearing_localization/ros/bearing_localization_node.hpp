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

/**
 * @brief ROS-level configuration parameters loaded from the ROS parameter
 * server.
 */
struct NodeConfig {
    std::string target_frame =
        "orca/odom";  // TF frame to transform measurements into.
    bool publish_debug_markers =
        true;               // Whether to publish debug visualisation markers.
    int landmark_type = 0;  // Landmark type ID in the output message.
    int landmark_subtype = 0;  // Landmark subtype ID in the output message.
};

/**
 * @brief ROS 2 node that performs bearing-only target localisation.
 *
 * Subscribes to single and array bearing measurements, transforms them into
 * the configured target frame via TF, feeds them to a BearingLocalizer, and
 * publishes estimated landmark positions.  Optionally publishes debug markers
 * for visualisation in Foxglove.
 */
class BearingLocalizationNode : public rclcpp::Node {
   public:
    explicit BearingLocalizationNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    /// @brief Declare all ROS parameters consumed by this node.
    void declare_parameters();

    /// @brief Create subscribers and publishers based on declared parameters.
    void setup_pubsub();

    /// @brief Callback for single Vector3Stamped bearing messages.
    void bearing_callback(
        const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

    /// @brief Callback for batched Vector3Array bearing messages.
    void bearing_array_callback(
        const vortex_msgs::msg::Vector3Array::SharedPtr msg);

    /**
     * @brief Transform a single bearing vector into the target frame and
     *        add it to the localiser buffer.
     * @return True if the measurement was accepted, false on TF failure.
     */
    bool process_bearing(const geometry_msgs::msg::Vector3& vec,
                         const std_msgs::msg::Header& header);

    /// @brief Publish the localisation result as a LandmarkArray message.
    void publish_result(const LocalizationResult& result);

    BearingLocalizationConfig cfg_;  // Algorithm config (from YAML profile).
    NodeConfig node_cfg_;            // ROS config (from parameter server).
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
