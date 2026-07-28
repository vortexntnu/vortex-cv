// TODO(member): implement HelloState.
//
// The header (include/vortex_yasmin_utils/hello_state.hpp) already declares
// what you have to write — treat it as the contract:
//
//   HelloState();
//   std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
//
// The includes below are the only ones you need. What's missing is the two
// definitions themselves, inside the vortex_yasmin_utils namespace:
//
//   - the constructor, which must tell yasmin::State which outcomes this
//     state is allowed to return (this one only ever returns SUCCEED), and
//   - execute(), which logs a greeting with YASMIN_LOG_INFO and returns that
//     same outcome.
//
// Outcome strings live in yasmin_ros::basic_outcomes (SUCCEED, ABORT,
// CANCEL). Use the constants, not raw string literals — a typo then fails at
// compile time instead of producing a machine that silently never
// transitions.
//
// For the shape of a finished minimal state, read WipeState in
// wipe_state.cpp in this same directory: the constructor passes its outcome
// set up to yasmin::State, execute() does its work and returns an outcome.
// Yours is simpler — no publisher, no node, just a log line.
//
// This file does not compile as-is, and that's expected: it isn't listed in
// CMakeLists.txt yet either (that's task 2). Delete this comment block when
// you're done.

#include "vortex_yasmin_utils/hello_state.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

namespace vortex_yasmin_utils {

// Your code goes here.

}  // namespace vortex_yasmin_utils
