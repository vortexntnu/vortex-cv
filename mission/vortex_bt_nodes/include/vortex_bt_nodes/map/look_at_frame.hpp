#ifndef VORTEX_BT_NODES__MAP__LOOK_AT_FRAME_HPP_
#define VORTEX_BT_NODES__MAP__LOOK_AT_FRAME_HPP_

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "vortex_bt_nodes/common/nav_action.hpp"
#include "vortex_bt_nodes/map/landmark_cache.hpp"

namespace vortex_bt_nodes::map {

/** @brief Turns to face a TF frame. FAILURE if it is not in TF. */
class LookAtFrame : public NavAction {
   public:
    LookAtFrame(const std::string& name,
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

#endif  // VORTEX_BT_NODES__MAP__LOOK_AT_FRAME_HPP_
