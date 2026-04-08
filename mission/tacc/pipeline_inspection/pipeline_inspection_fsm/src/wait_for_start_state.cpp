#include "pipeline_inspection_fsm/states.hpp"

WaitForStartState::WaitForStartState(yasmin::Blackboard::SharedPtr blackboard)
    : TriggerWaitState(blackboard->get<std::string>("service.start_mission")) {}
