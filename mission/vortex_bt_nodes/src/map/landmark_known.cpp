#include "vortex_bt_nodes/map/landmark_known.hpp"

namespace vortex_bt_nodes::map {
LandmarkKnown::LandmarkKnown(const std::string& name,
                             const BT::NodeConfig& config,
                             std::shared_ptr<const LandmarkMap> map)
    : BT::ConditionNode(name, config), map_(std::move(map)) {}

BT::PortsList LandmarkKnown::providedPorts() {
    return LandmarkMap::ports();
}

BT::NodeStatus LandmarkKnown::tick() {
    if (!map_) {
        return BT::NodeStatus::FAILURE;
    }

    if (map_->resolve(*this).has_value()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
}  // namespace vortex_bt_nodes::map
