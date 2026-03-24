#include "pipeline_inspection_fsm/states.hpp"
#include "pipeline_inspection_fsm/param_utils.hpp"

#include <yasmin_ros/yasmin_node.hpp>

#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

ConvergeState::ConvergeState(yasmin::Blackboard::SharedPtr)
    : ActionState(
          pipeline_inspection_fsm::param_utils::get_string(
              yasmin_ros::YasminNode::get_instance(),
              "action_servers.waypoint_manager"),
          std::bind(&ConvergeState::create_goal,
                    this,
                    std::placeholders::_1)) {
    auto node = yasmin_ros::YasminNode::get_instance();

    convergence_threshold_ =
        pipeline_inspection_fsm::param_utils::get_double(node, "fsm.convergence_threshold");

    const double offset_x =
        pipeline_inspection_fsm::param_utils::get_double(node, "fsm.landmark_offset.x");
    const double offset_y =
        pipeline_inspection_fsm::param_utils::get_double(node, "fsm.landmark_offset.y");
    const double offset_z =
        pipeline_inspection_fsm::param_utils::get_double(node, "fsm.landmark_offset.z");
    const double offset_roll =
        pipeline_inspection_fsm::param_utils::get_double(node, "fsm.landmark_offset.roll");
    const double offset_pitch =
        pipeline_inspection_fsm::param_utils::get_double(node, "fsm.landmark_offset.pitch");
    const double offset_yaw =
        pipeline_inspection_fsm::param_utils::get_double(node, "fsm.landmark_offset.yaw");

    pose_offset_ = vortex::utils::types::Pose::from_eigen(
        Eigen::Vector3d{offset_x, offset_y, offset_z},
        vortex::utils::math::euler_to_quat(offset_roll, offset_pitch,
                                           offset_yaw));
}

pipeline_inspection_fsm::WaypointManagerAction::Goal ConvergeState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    auto landmark =
        blackboard->get<vortex_msgs::msg::Landmark>("found_landmark");

    const auto landmark_pose =
        vortex::utils::ros_conversions::ros_pose_to_pose(landmark.pose.pose);
    const auto target_pose = vortex::utils::waypoints::apply_pose_offset(
        landmark_pose, pose_offset_);

    vortex_msgs::msg::Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::to_pose_msg(target_pose);
    wp.waypoint_mode.mode = vortex_msgs::msg::WaypointMode::ONLY_POSITION;

    pipeline_inspection_fsm::WaypointManagerAction::Goal goal;
    goal.waypoints = {wp};
    goal.persistent = false;
    goal.convergence_threshold = convergence_threshold_;

    return goal;
}
