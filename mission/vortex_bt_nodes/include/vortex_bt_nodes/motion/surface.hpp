#ifndef VORTEX_BT_NODES__MOTION__SURFACE_HPP_
#define VORTEX_BT_NODES__MOTION__SURFACE_HPP_

#include <optional>
#include <string>

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::motion {

/**
 * @brief Goes up to depth z (default 0.2 m) like SetDepth: a separate name so
 * the tree reads as it does.
 */
class Surface : public NavAction {
   public:
    using NavAction::NavAction;

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__SURFACE_HPP_
