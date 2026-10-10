#ifndef VORTEX_BT_NODES__MAP__TURN_HPP_
#define VORTEX_BT_NODES__MAP__TURN_HPP_

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "vortex_bt_nodes/common/nav_action.hpp"
#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Turns to yaw_deg in the map frame, or by relative_deg from the
 * current heading. Give one of the two.
 */
class Turn : public NavAction {
   public:
    Turn(const std::string& name,
         const BT::NodeConfig& config,
         rclcpp::Node::SharedPtr node,
         std::shared_ptr<const LandmarkCache> cache)
        : NavAction(name, config, std::move(node)), cache_(std::move(cache)) {}

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;

   private:
    std::shared_ptr<const LandmarkCache> cache_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__TURN_HPP_
