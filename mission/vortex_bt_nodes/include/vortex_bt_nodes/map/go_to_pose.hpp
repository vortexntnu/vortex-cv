#ifndef VORTEX_BT_NODES__MAP__GO_TO_POSE_HPP_
#define VORTEX_BT_NODES__MAP__GO_TO_POSE_HPP_

#include <optional>

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Goes to a pose from the blackboard (odom), plus an offset in that
 * pose's frame, and never updates the goal: the pose is one CommitTarget
 * froze. offset "-1;0;0" is 1 m in front of it, "1;0;0" 1 m behind, facing
 * along its +X.
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
