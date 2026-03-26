#ifndef VORTEX_YASMIN_UTILS__SERVICE_REQUEST_WAIT_STATE_HPP_
#define VORTEX_YASMIN_UTILS__SERVICE_REQUEST_WAIT_STATE_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <yasmin/state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/yasmin_node.hpp>

namespace vortex_yasmin_utils {

/**
 * @brief A YASMIN state that blocks until a ROS service of type ServiceT is
 * called, then stores the request on the blackboard.
 *
 * This is a generic version of ServiceTriggerWaitState that works with any service
 * type, making the service request data available to downstream states via the
 * blackboard.
 *
 * @tparam ServiceT  The ROS service type (e.g. your_msgs::srv::SetPose).
 *                   The response type must have a `success` bool field.
 *
 * @param service_name    The fully-qualified ROS service name to advertise.
 * @param request_bb_key  Blackboard key under which the request is stored.
 * @param timeout         How long to wait before timing out.
 *                        A duration of zero (default) means wait indefinitely.
 */
template <typename ServiceT>
class ServiceRequestWaitState : public yasmin::State {
   public:
    explicit ServiceRequestWaitState(
        const std::string& service_name,
        const std::string& request_bb_key,
        std::chrono::duration<double> timeout =
            std::chrono::duration<double>(0))
        : yasmin::State(make_outcomes(timeout.count() > 0)),
          request_bb_key_(request_bb_key),
          timeout_(timeout) {
        auto node = yasmin_ros::YasminNode::get_instance();

        service_ = node->create_service<ServiceT>(
            service_name,
            std::bind(&ServiceRequestWaitState::service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
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

        blackboard->set<typename ServiceT::Request::SharedPtr>(
            request_bb_key_, last_request_);

        on_triggered(blackboard, last_request_);
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    void cancel_state() override {
        yasmin::State::cancel_state();
        cv_.notify_all();
    }

    static constexpr const char* TIMEOUT = "timeout";

   protected:
    virtual void on_triggered(
        yasmin::Blackboard::SharedPtr blackboard,
        typename ServiceT::Request::SharedPtr request) {
        (void)blackboard;
        (void)request;
    }

   private:
    static std::set<std::string> make_outcomes(bool has_timeout) {
        std::set<std::string> outcomes{yasmin_ros::basic_outcomes::SUCCEED,
                                       yasmin_ros::basic_outcomes::CANCEL};
        if (has_timeout)
            outcomes.insert(TIMEOUT);
        return outcomes;
    }

    void service_callback(
        typename ServiceT::Request::SharedPtr req,
        typename ServiceT::Response::SharedPtr res) {
        std::lock_guard lock(mutex_);

        last_request_ = req;
        triggered_ = true;
        res->success = true;

        cv_.notify_all();
    }

    typename rclcpp::Service<ServiceT>::SharedPtr service_;
    typename ServiceT::Request::SharedPtr last_request_;
    std::string request_bb_key_;
    std::condition_variable cv_;
    std::mutex mutex_;
    bool triggered_{false};
    std::chrono::duration<double> timeout_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__SERVICE_REQUEST_WAIT_STATE_HPP_
