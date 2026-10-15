#ifndef VORTEX_BT_NODES__MAP__GO_TO_FRAME_NODE_HPP_
#define VORTEX_BT_NODES__MAP__GO_TO_FRAME_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "vortex_bt_nodes/common/nav_action.hpp"
#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Goes to a TF frame plus an offset in that frame, and follows the
 * frame while it moves in odom.
 *
 * FAILURE when the frame, or tool_frame, is not in TF when the node starts.
 */
class GoToFrame : public NavAction {
   public:
    GoToFrame(const std::string& name,
              const BT::NodeConfig& config,
              rclcpp::Node::SharedPtr node,
              std::shared_ptr<const LandmarkMap> map);

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
    std::optional<Goal> update_goal() override;

   private:
    /** @brief frame + offset in odom. nullopt when a frame is not in TF. */
    std::optional<Pose> target_in_odom() const;

    /** @brief Waypoints from where the vehicle is to target, split by
     * max_step_m. */
    std::optional<Goal> build_goal(const Pose& target) const;

    std::shared_ptr<const LandmarkMap> map_;
    // The target of the goal that is running, in odom.
    Pose last_target_;
    // Set once the vehicle is inside dead_reckoning_threshold: the target is
    // then kept as it is, the detections close up are too poor to follow.
    bool frozen_{false};
    // Only used to log a lost frame once instead of every tick.
    bool lost_{false};
};  // GoToFrame

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__GO_TO_FRAME_NODE_HPP_
