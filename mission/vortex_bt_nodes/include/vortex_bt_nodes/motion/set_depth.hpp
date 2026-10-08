#ifndef VORTEX_BT_NODES__MOTION__SET_DEPTH_HPP_
#define VORTEX_BT_NODES__MOTION__SET_DEPTH_HPP_

#include <optional>
#include <string>

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::motion {

/**
 * @brief Goes to depth z (odom z, down positive), keeps x, y and heading
 * (mode ONLY_Z).
 */
class SetDepth : public NavAction {
   public:
    using NavAction::NavAction;

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__SET_DEPTH_HPP_
