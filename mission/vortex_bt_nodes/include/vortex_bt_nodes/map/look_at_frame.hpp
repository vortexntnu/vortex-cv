#ifndef VORTEX_BT_NODES__MAP__LOOK_AT_FRAME_HPP_
#define VORTEX_BT_NODES__MAP__LOOK_AT_FRAME_HPP_

#include <memory>
#include <optional>
#include <string>

#include "vortex_bt_nodes/common/nav_action.hpp"
#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::map {

/**
 * @brief Turns on the spot to face a TF frame. FAILURE if the frame or the
 * vehicle is not in TF, or the frame is straight above or below the vehicle.
 */
class LookAtFrame : public NavAction {
   public:
    LookAtFrame(const std::string& name,
                const BT::NodeConfig& config,
                rclcpp::Node::SharedPtr node,
                std::shared_ptr<const LandmarkMap> map);

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;

   private:
    std::shared_ptr<const LandmarkMap> map_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__LOOK_AT_FRAME_HPP_
