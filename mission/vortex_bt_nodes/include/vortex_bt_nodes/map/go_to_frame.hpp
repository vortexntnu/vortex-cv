#ifndef VORTEX_BT_NODES__MAP__GO_TO_FRAME_HPP_
#define VORTEX_BT_NODES__MAP__GO_TO_FRAME_HPP_

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "vortex_bt_nodes/common/nav_action.hpp"
#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Goes to a TF frame plus an offset in that frame. The goal is sent
 * again when the frame moves more than resend_m, until the vehicle is within
 * freeze_within_m. FAILURE if the frame is not in TF at the start.
 */
class GoToFrame : public NavAction {
   public:
    GoToFrame(const std::string& name,
              const BT::NodeConfig& config,
              rclcpp::Node::SharedPtr node,
              std::shared_ptr<const LandmarkCache> cache)
        : NavAction(name, config, std::move(node)), cache_(std::move(cache)) {}

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
    std::optional<Goal> update_goal() override;

   private:
    std::optional<Pose> target() const;
    std::optional<Goal> goal_for(const Pose& target) const;

    std::shared_ptr<const LandmarkCache> cache_;
    Pose sent_target_;
    bool frozen_{false};
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__GO_TO_FRAME_HPP_
