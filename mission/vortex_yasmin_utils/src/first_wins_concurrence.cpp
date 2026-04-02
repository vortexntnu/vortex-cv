#include "vortex_yasmin_utils/first_wins_concurrence.hpp"

#include <stdexcept>
#include <thread>
#include <vector>

#include <yasmin/logs.hpp>
#include <yasmin_ros/basic_outcomes.hpp>

namespace vortex_yasmin_utils {

// static
yasmin::Outcomes FirstWinsConcurrence::collect_outcomes(
    const FirstWinsOutcomeMap& outcome_map,
    const std::string& default_outcome) {
    yasmin::Outcomes outcomes;
    outcomes.insert(default_outcome);
    outcomes.insert(yasmin_ros::basic_outcomes::CANCEL);
    for (const auto& [state_name, state_outcomes] : outcome_map) {
        for (const auto& [state_outcome, overall_outcome] : state_outcomes) {
            outcomes.insert(overall_outcome);
        }
    }
    return outcomes;
}

FirstWinsConcurrence::FirstWinsConcurrence(
    const yasmin::StateMap& states,
    const std::string& default_outcome,
    const FirstWinsOutcomeMap& outcome_map)
    : State(collect_outcomes(outcome_map, default_outcome)),
      states_(states),
      default_outcome_(default_outcome),
      outcome_map_(outcome_map) {
    for (const auto& [state_name, _] : outcome_map) {
        if (states.find(state_name) == states.end()) {
            throw std::invalid_argument(
                "FirstWinsConcurrence: outcome_map references unknown state '" +
                state_name + "'");
        }
    }
}

std::string FirstWinsConcurrence::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
    first_done_.store(false);
    winning_state_name_.clear();
    winning_state_outcome_.clear();

    std::vector<std::thread> threads;
    threads.reserve(states_.size());

    for (const auto& [name, state] : states_) {
        threads.emplace_back([this, blackboard, name = name, state = state]() {
            const std::string outcome = (*state)(blackboard);

            // Only the first completion wins; subsequent ones are ignored.
            bool expected = false;
            if (first_done_.compare_exchange_strong(expected, true)) {
                {
                    std::lock_guard<std::mutex> lock(notify_mutex_);
                    winning_state_name_ = name;
                    winning_state_outcome_ = outcome;
                }
                notify_cv_.notify_one();
            }
        });
    }

    // Block until the first state completes or we are externally cancelled.
    {
        std::unique_lock<std::mutex> lock(notify_mutex_);
        notify_cv_.wait(
            lock, [this] { return first_done_.load() || this->is_canceled(); });
    }

    // Cancel every state that is still running.
    for (const auto& [name, state] : states_) {
        if (state->is_running()) {
            YASMIN_LOG_INFO(
                "FirstWinsConcurrence: cancelling '%s' because another state "
                "finished first",
                name.c_str());
            state->cancel_state();
        }
    }

    for (auto& t : threads) {
        t.join();
    }

    if (this->is_canceled()) {
        return yasmin_ros::basic_outcomes::CANCEL;
    }

    // Translate the winner's outcome to an overall outcome.
    const auto state_it = outcome_map_.find(winning_state_name_);
    if (state_it != outcome_map_.end()) {
        const auto outcome_it = state_it->second.find(winning_state_outcome_);
        if (outcome_it != state_it->second.end()) {
            return outcome_it->second;
        }
    }

    YASMIN_LOG_WARN(
        "FirstWinsConcurrence: no mapping for state '%s' outcome '%s', "
        "returning default '%s'",
        winning_state_name_.c_str(), winning_state_outcome_.c_str(),
        default_outcome_.c_str());
    return default_outcome_;
}

void FirstWinsConcurrence::cancel_state() {
    // Set the canceled flag first so execute()'s wait condition sees it.
    yasmin::State::cancel_state();

    notify_cv_.notify_all();

    for (const auto& [name, state] : states_) {
        if (state->is_running()) {
            state->cancel_state();
        }
    }
}

}  // namespace vortex_yasmin_utils
