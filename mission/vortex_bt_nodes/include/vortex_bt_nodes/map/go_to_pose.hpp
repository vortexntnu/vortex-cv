#ifndef VORTEX_BT_NODES__MAP__GO_TO_POSE_HPP_
#define VORTEX_BT_NODES__MAP__GO_TO_POSE_HPP_

#include <optional>

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Goes to a blackboard pose plus an offset in that pose's frame. The
 * goal is never updated. Offset "-1;0;0" is 1 m in front of the pose.
 */
class GoToPose : public NavAction {
   public:
    using NavAction::NavAction;

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__GO_TO_POSE_HPP_
