#ifndef BEARING_DIRECTION_SERVER__ACOUSTICS_BEARING_NODE_HPP_
#define BEARING_DIRECTION_SERVER__ACOUSTICS_BEARING_NODE_HPP_

#include "bearing_direction_server/bearing_direction_base.hpp"

#include <optional>
#include <Eigen/Dense>
#include <vortex_msgs/msg/bearing_measurement.hpp>

namespace bearing_direction_server {

/**
 * Subscribes to BearingMeasurement (acoustic pinger).
 * Rotates the direction vector into the odom frame via TF, then applies
 * a weighted-average filter before committing to the base class buffer.
 */
class AcousticsBearingNode : public BearingDirectionBase {
   public:
    explicit AcousticsBearingNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void on_collection_start() override { filter_ = FilterState{}; }

    void bearing_callback(
        const vortex_msgs::msg::BearingMeasurement::SharedPtr msg);

    rclcpp::Subscription<vortex_msgs::msg::BearingMeasurement>::SharedPtr
        bearing_sub_;

    struct FilterState {
        Eigen::Vector3d dir{0.0, 0.0, 0.0};
        Eigen::Vector3d pos{0.0, 0.0, 0.0};
        double weight{0.0};
    };
    FilterState filter_;
};

}  // namespace bearing_direction_server

#endif  // BEARING_DIRECTION_SERVER__ACOUSTICS_BEARING_NODE_HPP_
