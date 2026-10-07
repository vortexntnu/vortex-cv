#include "vortex_bt_nodes/map/landmark_confirmed.hpp"

namespace vortex_bt_nodes::map {

BT::PortsList LandmarkConfirmed::providedPorts() {
    BT::PortsList ports = LandmarkCache::ports();
    ports.insert(BT::InputPort<double>("max_sigma_xy", 0.3,
                                       "Horizontal position std limit [m]"));
    return ports;
}

BT::NodeStatus LandmarkConfirmed::tick() {
    const auto landmark = cache_->resolve(*this);
    const double max_sigma = getInput<double>("max_sigma_xy").value_or(0.3);
    return landmark && LandmarkCache::confirmed(*landmark, max_sigma)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

}  // namespace vortex_bt_nodes::map
