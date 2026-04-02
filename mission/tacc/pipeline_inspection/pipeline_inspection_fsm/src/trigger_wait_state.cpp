#include "pipeline_inspection_fsm/states.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/yasmin_node.hpp>

TriggerWaitState::TriggerWaitState(const std::string& service_name)
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::CANCEL}) {
    auto node = yasmin_ros::YasminNode::get_instance();

    service_ = node->create_service<pipeline_inspection_fsm::TriggerSrv>(
        service_name, std::bind(&TriggerWaitState::callback, this,
                                std::placeholders::_1, std::placeholders::_2));
}

std::string TriggerWaitState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
    {
        std::lock_guard lock(mutex_);
        triggered_ = false;
    }

    std::unique_lock lock(mutex_);

    cv_.wait(lock, [this] { return triggered_ || is_canceled(); });

    if (is_canceled())
        return yasmin_ros::basic_outcomes::CANCEL;

    on_triggered(blackboard);
    return yasmin_ros::basic_outcomes::SUCCEED;
}

void TriggerWaitState::cancel_state() {
    cv_.notify_all();
    yasmin::State::cancel_state();
}

void TriggerWaitState::on_triggered(yasmin::Blackboard::SharedPtr) {}

void TriggerWaitState::callback(
    pipeline_inspection_fsm::TriggerSrv::Request::SharedPtr,
    pipeline_inspection_fsm::TriggerSrv::Response::SharedPtr res) {
    std::lock_guard lock(mutex_);

    triggered_ = true;
    res->success = true;

    cv_.notify_all();
}
