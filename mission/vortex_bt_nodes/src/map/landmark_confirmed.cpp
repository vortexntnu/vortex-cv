#include "vortex_bt_nodes/map/landmark_confirmed.hpp"

#include <spdlog/spdlog.h>

namespace vortex_bt_nodes::map {

LandmarkConfirmed::LandmarkConfirmed(const std::string& name,
                                     const BT::NodeConfig& config,
                                     std::shared_ptr<const LandmarkMap> map)
    : BT::ConditionNode(name, config), map_(std::move(map)) {}

BT::PortsList LandmarkConfirmed::providedPorts() {
    BT::PortsList ports = LandmarkMap::ports();
    ports.insert(BT::InputPort<double>(
        "max_sigma_xy", "Largest horizontal std that counts as confirmed [m]"));
    return ports;
}

BT::NodeStatus LandmarkConfirmed::tick() {
    const auto max_sigma_xy = getInput<double>("max_sigma_xy");
    if (!max_sigma_xy || *max_sigma_xy <= 0.0) {
        SPDLOG_ERROR("LandmarkConfirmed: max_sigma_xy missing or not positive");
        return BT::NodeStatus::FAILURE;
    }

    const auto landmark = map_->resolve(*this);
    if (!landmark) {
        return BT::NodeStatus::FAILURE;
    }

    return LandmarkMap::confirmed(*landmark, *max_sigma_xy)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

}  // namespace vortex_bt_nodes::map
