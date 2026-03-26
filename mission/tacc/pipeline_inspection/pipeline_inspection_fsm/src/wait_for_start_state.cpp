#include "pipeline_inspection_fsm/states.hpp"

WaitForStartState::WaitForStartState(yasmin::Blackboard::SharedPtr blackboard)
    : vortex_yasmin_utils::ServiceTriggerWaitState(
          blackboard->get<std::string>("service.start_mission")) {}
