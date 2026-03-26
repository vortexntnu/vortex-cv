#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/yasmin_node.hpp>

namespace vortex_yasmin_utils {

static std::set<std::string> make_outcomes(bool has_timeout) {
    std::set<std::string> outcomes{yasmin_ros::basic_outcomes::SUCCEED,
                                   yasmin_ros::basic_outcomes::CANCEL};
    if (has_timeout)
        outcomes.insert(ServiceTriggerWaitState::TIMEOUT);
    return outcomes;
}

ServiceTriggerWaitState::ServiceTriggerWaitState(const std::string& service_name,
                                   std::chrono::duration<double> timeout)
    : yasmin::State(make_outcomes(timeout.count() > 0)), timeout_(timeout) {
    auto node = yasmin_ros::YasminNode::get_instance();

    service_ = node->create_service<std_srvs::srv::Trigger>(
        service_name, std::bind(&ServiceTriggerWaitState::service_callback, this,
                                std::placeholders::_1, std::placeholders::_2));
}

std::string ServiceTriggerWaitState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
    {
        std::lock_guard lock(mutex_);
        triggered_ = false;
    }

    std::unique_lock lock(mutex_);

    auto predicate = [this] { return triggered_ || is_canceled(); };

    if (timeout_.count() > 0) {
        if (!cv_.wait_for(lock, timeout_, predicate)) {
            return TIMEOUT;
        }
    } else {
        cv_.wait(lock, predicate);
    }

    if (is_canceled())
        return yasmin_ros::basic_outcomes::CANCEL;

    on_triggered(blackboard);
    return yasmin_ros::basic_outcomes::SUCCEED;
}

void ServiceTriggerWaitState::cancel_state() {
    yasmin::State::cancel_state();
    cv_.notify_all();
}

void ServiceTriggerWaitState::on_triggered(yasmin::Blackboard::SharedPtr) {}

void ServiceTriggerWaitState::service_callback(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
    std::lock_guard lock(mutex_);

    triggered_ = true;
    res->success = true;

    cv_.notify_all();
}

}  // namespace vortex_yasmin_utils
