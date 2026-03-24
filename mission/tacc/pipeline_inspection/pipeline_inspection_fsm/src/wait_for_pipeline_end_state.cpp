#include "pipeline_inspection_fsm/states.hpp"
#include "pipeline_inspection_fsm/param_utils.hpp"

#include <yasmin_ros/yasmin_node.hpp>

WaitForPipelineEndState::WaitForPipelineEndState(yasmin::Blackboard::SharedPtr)
    : TriggerWaitState(
          pipeline_inspection_fsm::param_utils::get_string(
              yasmin_ros::YasminNode::get_instance(),
              "services.end_of_pipeline")) {}

void WaitForPipelineEndState::on_triggered(
    yasmin::Blackboard::SharedPtr blackboard) {
    auto wm_handle =
        blackboard->get<pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>(
            "wm_handle");

    if (!wm_handle)
        YASMIN_LOG_WARN(
            "WaitForPipelineEndState: no active WM handle at pipeline end");
}
