#include "vortex_bt_nodes/map/landmark_known.hpp"

namespace vortex_bt_nodes::map {

BT::PortsList LandmarkKnown::providedPorts() {
    return LandmarkCache::ports();
}

BT::NodeStatus LandmarkKnown::tick() {
    return cache_->resolve(*this) ? BT::NodeStatus::SUCCESS
                                  : BT::NodeStatus::FAILURE;
}

}  // namespace vortex_bt_nodes::map
