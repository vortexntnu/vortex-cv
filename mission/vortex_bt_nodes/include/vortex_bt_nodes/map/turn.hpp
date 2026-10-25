#ifndef VORTEX_BT_NODES__MAP__TURN_HPP_
#define VORTEX_BT_NODES__MAP__TURN_HPP_

#include <memory>
#include <optional>
#include <string>

#include "vortex_bt_nodes/common/nav_action.hpp"
#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Turns on the spot to a heading in the map frame (yaw_deg), or by an
 * angle from the current heading (relative_deg). Exactly one must be set.
 */
class Turn : public NavAction {
   public:
    Turn(const std::string& name,
         const BT::NodeConfig& config,
         rclcpp::Node::SharedPtr node,
         std::shared_ptr<const LandmarkMap> map);

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;

   private:
    std::shared_ptr<const LandmarkMap> map_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__TURN_HPP_
