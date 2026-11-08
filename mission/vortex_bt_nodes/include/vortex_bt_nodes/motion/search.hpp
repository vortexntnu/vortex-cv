#ifndef VORTEX_BT_NODES__MOTION__SEARCH_HPP_
#define VORTEX_BT_NODES__MOTION__SEARCH_HPP_

#include "vortex_bt_nodes/common/nav_action.hpp"
#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::motion {

class Search : public NavAction {
   public:
    Search(const std::string& name,
           const BT::NodeConfig& config,
           rclcpp::Node::SharedPtr node,
           std::shared_ptr<const map::LandmarkMap> map);

    static BT::PortsList providedPorts();

   protected:
    std::optional<Goal> make_goal() override;
    BT::NodeStatus on_result(const GoalHandle::WrappedResult& result) override;

   private:
    std::shared_ptr<const map::LandmarkMap> map_;
};

}  // namespace vortex_bt_nodes::motion

#endif  // VORTEX_BT_NODES__MOTION__SEARCH_HPP_
