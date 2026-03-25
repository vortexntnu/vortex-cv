#include "pipeline_inspection_fsm/states.hpp"

WaitForStartState::WaitForStartState(yasmin::Blackboard::SharedPtr blackboard)
    : vortex_yasmin_utils::TriggerWaitState(
          blackboard->get<std::string>("service.start_mission")) {}
