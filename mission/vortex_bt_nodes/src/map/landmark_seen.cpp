#include "vortex_bt_nodes/map/landmark_seen.hpp"

namespace vortex_bt_nodes::map {

LandmarkSeen::LandmarkSeen(const std::string& name,
                           const BT::NodeConfig& config,
                           std::shared_ptr<const LandmarkMap> map)
    : BT::ConditionNode(name, config), map_(std::move(map)) {}

BT::PortsList LandmarkSeen::providedPorts() {
    BT::PortsList ports = LandmarkMap::ports();
    ports.insert(BT::InputPort<double>("within_s", 2.0,
                                       "Newest detection at most this old [s]"));
    return ports;
}

BT::NodeStatus LandmarkSeen::tick() {
    const auto ago = map_->seen_ago(*this);
    return ago && *ago <= getInput<double>("within_s").value_or(2.0)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

}  // namespace vortex_bt_nodes::map
