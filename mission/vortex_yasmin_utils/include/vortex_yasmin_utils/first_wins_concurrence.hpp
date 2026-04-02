#ifndef VORTEX_YASMIN_UTILS__FIRST_WINS_CONCURRENCE_HPP_
#define VORTEX_YASMIN_UTILS__FIRST_WINS_CONCURRENCE_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <unordered_map>

#include <yasmin/state.hpp>
#include <yasmin/types.hpp>

namespace vortex_yasmin_utils {

/**
 * @brief Maps each concurrent state's name to its own outcome->overall_outcome
 * table.
 *
 * Example:
 * @code
 * FirstWinsOutcomeMap om = {
 *     {"WAYPOINT_GOAL",     {{SUCCEED, "waypoint_reached"}, {ABORT, ABORT}}},
 *     {"LANDMARK_POLLING",  {{"landmarks_found", "landmarks_found"}, {ABORT,
 * ABORT}}},
 * };
 * @endcode
 */
using FirstWinsOutcomeMap =
    std::unordered_map<std::string,
                       std::unordered_map<std::string, std::string>>;

/**
 * @brief Runs a set of states concurrently; the first to finish cancels all
 * others.
 *
 * Unlike yasmin::Concurrence (which waits for every state to complete),
 * FirstWinsConcurrence returns as soon as any one state produces an outcome.
 * Every still-running state is immediately cancelled and the class waits for
 * their threads to join before returning.
 *
 * The overall outcome is resolved by looking up
 * outcome_map[winning_state_name][winning_state_outcome].
 * If no entry is found, default_outcome is returned.
 *
 * If the concurrence itself is externally cancelled (via cancel_state()),
 * all child states are cancelled and CANCEL is returned.
 *
 * @param states         Map of logical name → state to run concurrently.
 * @param default_outcome Returned when the winner's outcome has no mapping.
 * @param outcome_map    Per-state outcome translation table (see above).
 */
class FirstWinsConcurrence : public yasmin::State {
   public:
    YASMIN_PTR_ALIASES(FirstWinsConcurrence)

    FirstWinsConcurrence(const yasmin::StateMap& states,
                         const std::string& default_outcome,
                         const FirstWinsOutcomeMap& outcome_map);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

    void cancel_state() override;

   private:
    const yasmin::StateMap states_;
    const std::string default_outcome_;
    const FirstWinsOutcomeMap outcome_map_;

    std::atomic<bool> first_done_{false};
    std::string winning_state_name_;
    std::string winning_state_outcome_;
    std::condition_variable notify_cv_;
    std::mutex notify_mutex_;

    static yasmin::Outcomes collect_outcomes(
        const FirstWinsOutcomeMap& outcome_map,
        const std::string& default_outcome);
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__FIRST_WINS_CONCURRENCE_HPP_
