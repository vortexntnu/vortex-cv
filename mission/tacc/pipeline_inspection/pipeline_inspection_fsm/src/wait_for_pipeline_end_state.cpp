#include "pipeline_inspection_fsm/states.hpp"

WaitForPipelineEndState::WaitForPipelineEndState(
    yasmin::Blackboard::SharedPtr blackboard)
    : vortex_yasmin_utils::TriggerWaitState(
          blackboard->get<std::string>("service.end_of_pipeline")) {}

void WaitForPipelineEndState::on_triggered(
    yasmin::Blackboard::SharedPtr blackboard) {
    auto wm_handle = blackboard->get<
        pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>(
        "wm_handle");

    if (!wm_handle)
        YASMIN_LOG_WARN(
            "WaitForPipelineEndState: no active WM handle at pipeline end");
}
