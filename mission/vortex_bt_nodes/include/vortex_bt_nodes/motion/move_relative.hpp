#ifndef VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_HPP_
#define VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_HPP_

#include <optional>
#include <string>

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::motion {

/**
 * @brief Moves by offset "x;y;z[;yaw_deg]" from the pose at start, in the
 * vehicle frame (BODY) or along the odom axes (WORLD).
 */
class MoveRelative : public NavAction {
   public:
    using NavAction::NavAction;

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__MOVE_RELATIVE_HPP_
