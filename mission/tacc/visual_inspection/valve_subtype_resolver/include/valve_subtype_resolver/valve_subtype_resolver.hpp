#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vortex_msgs/msg/landmark_array.hpp>

#include <memory>
#include <string>

namespace valve_subtype_resolver {

class ValveSubtypeResolverNode : public rclcpp::Node {
   public:
    explicit ValveSubtypeResolverNode(const rclcpp::NodeOptions& options);

   private:
    void landmarks_cb(const vortex_msgs::msg::LandmarkArray::SharedPtr msg);

    // Uses the TF tree to rotate the valve plane normal into the world frame,
    // then returns VALVE_VERTICAL (1), VALVE_HORIZONTAL (2), or 0 (unknown).
    int resolve_subtype(const vortex_msgs::msg::Landmark& landmark);

    // Folds the landmark's yaw (measured in the world frame) into [0, π/2].
    // Leaves pitch/roll intact. Mutates `lm.pose.pose.orientation` in place.
    // No-op if the required TF transform is unavailable.
    void fold_yaw_world(vortex_msgs::msg::Landmark& lm);

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
        landmark_sub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;

    std::string world_frame_;

    // Threshold on |world_normal.z|: above this the valve is VALVE_VERTICAL
    // (normal points up/down), below it VALVE_HORIZONTAL (normal in XY plane).
    double vertical_threshold_;

    // Fold the landmark yaw into [0, π/2] in the world frame so the result
    // is drone-roll invariant (valve handle is 180°/90° symmetric).
    bool clamp_yaw_{false};
};

}  // namespace valve_subtype_resolver
