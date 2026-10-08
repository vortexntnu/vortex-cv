#ifndef VORTEX_BT_NODES__MOTION__SEARCH_HPP_
#define VORTEX_BT_NODES__MOTION__SEARCH_HPP_

#include <optional>
#include <string>

#include "vortex_bt_nodes/common/nav_action.hpp"

namespace vortex_bt_nodes::motion {

/**
 * @brief Turns on the spot to look for something: ROTATE_STEPS (a full turn
 * in step_deg steps) or SCAN_ARC (arc_deg to each side and back), holding
 * pause_s at each heading so the detectors can see. The tree stops it when
 * the target is found (e.g. ReactiveFallback with LandmarkConfirmed);
 * returns FAILURE when the sweep is done without being stopped.
 */
class Search : public NavAction {
   public:
    using NavAction::NavAction;

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
    BT::NodeStatus on_result(const GoalHandle::WrappedResult& result) override;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__SEARCH_HPP_
