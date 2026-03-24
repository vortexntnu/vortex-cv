#include "pipeline_inspection_fsm/states.hpp"

std::shared_ptr<yasmin::Blackboard> initialize_blackboard() {
    auto bb = std::make_shared<yasmin::Blackboard>();

    bb->set<bool>("landmark_found", false);
    bb->set<pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>(
        "wm_handle", nullptr);

    return bb;
}
