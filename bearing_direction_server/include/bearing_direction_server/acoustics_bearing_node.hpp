#ifndef BEARING_DIRECTION_SERVER__ACOUSTICS_BEARING_NODE_HPP_
#define BEARING_DIRECTION_SERVER__ACOUSTICS_BEARING_NODE_HPP_

#include "bearing_direction_server/bearing_direction_base.hpp"

#include <vortex_msgs/msg/bearing_measurement_array.hpp>

namespace bearing_direction_server {

/**
 * Subscribes to BearingMeasurementArray (acoustic pinger).
 * Rotates each direction vector into the odom frame via TF.
 */
class AcousticsBearingNode : public BearingDirectionBase {
   public:
    explicit AcousticsBearingNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void bearing_callback(
        const vortex_msgs::msg::BearingMeasurementArray::SharedPtr msg);

    rclcpp::Subscription<vortex_msgs::msg::BearingMeasurementArray>::SharedPtr
        bearing_sub_;
};

}  // namespace bearing_direction_server

#endif  // BEARING_DIRECTION_SERVER__ACOUSTICS_BEARING_NODE_HPP_
