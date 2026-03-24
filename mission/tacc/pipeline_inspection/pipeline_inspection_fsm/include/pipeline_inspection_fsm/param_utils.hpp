#ifndef PIPELINE_INSPECTION_FSM__PARAM_UTILS_HPP_
#define PIPELINE_INSPECTION_FSM__PARAM_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace pipeline_inspection_fsm::param_utils {

inline std::string get_string(const rclcpp::Node::SharedPtr& node,
                              const std::string& name) {
    if (!node->has_parameter(name))
        node->declare_parameter<std::string>(name);
    return node->get_parameter(name).as_string();
}

inline int get_int(const rclcpp::Node::SharedPtr& node,
                   const std::string& name) {
    if (!node->has_parameter(name))
        node->declare_parameter<int>(name);
    return node->get_parameter(name).as_int();
}

inline double get_double(const rclcpp::Node::SharedPtr& node,
                         const std::string& name) {
    if (!node->has_parameter(name))
        node->declare_parameter<double>(name);
    return node->get_parameter(name).as_double();
}

}  // namespace pipeline_inspection_fsm::param_utils

#endif  // PIPELINE_INSPECTION_FSM__PARAM_UTILS_HPP_
