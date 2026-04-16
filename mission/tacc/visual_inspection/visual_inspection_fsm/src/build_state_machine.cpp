#include "valve_inspection_fsm/states.hpp"

#include <cmath>

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>

#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

using vortex::utils::types::Pose;
using vortex::utils::waypoints::WaypointGoal;
using vortex_yasmin_utils::LandmarkPollingState;
using vortex_yasmin_utils::ServiceTriggerWaitState;
using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/// @brief ActionState that reads a WaypointGoal from the blackboard and sends
/// it to the WaypointManager. Used for dynamically computed waypoints
/// (approach/retreat) where the pose is calculated at runtime.
class BlackboardWaypointState
    : public yasmin_ros::ActionState<WaypointManagerAction> {
   public:
    BlackboardWaypointState(const std::string& action_server_name,
                            std::string waypoint_goal_bb_key)
        : ActionState(action_server_name,
                      std::bind(&BlackboardWaypointState::create_goal,
                                this,
                                std::placeholders::_1)),
          waypoint_goal_bb_key_(std::move(waypoint_goal_bb_key)) {}

    WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard) {
        auto waypoint_goal =
            blackboard->get<WaypointGoal>(waypoint_goal_bb_key_);

        vortex_msgs::msg::Waypoint wp;
        wp.pose =
            vortex::utils::ros_conversions::to_pose_msg(waypoint_goal.pose);
        wp.waypoint_mode.mode = static_cast<uint8_t>(waypoint_goal.mode);

        WaypointManagerAction::Goal goal;
        goal.waypoints = {wp};
        goal.persistent = false;
        goal.convergence_threshold = waypoint_goal.convergence_threshold;

        return goal;
    }

   private:
    std::string waypoint_goal_bb_key_;
};

/// @brief Compute gripper roll servo angle to align fingers with valve handle.
///
/// The residual rotation q_drone^{-1} * q_valve captures the mismatch
/// between the drone's orientation and the valve frame. The x-axis roll
/// component of this residual is the angle the gripper servo must rotate.
static double compute_gripper_roll(const Eigen::Quaterniond& q_drone,
                                   const Eigen::Quaterniond& q_valve) {
    const Eigen::Matrix3d R =
        (q_drone.inverse() * q_valve).normalized().toRotationMatrix();
    return std::atan2(R(2, 1), R(1, 1));
}

/// @brief Compute a standoff waypoint along the approach direction defined by
/// the valve handle yaw. The drone is positioned standoff_distance metres from
/// the valve along that direction and oriented to face back at the valve
/// (yaw + 180°).
///
/// Standoff distance = norm of standoff_goal.pose position vector.
///
/// @param landmarks_bb_key      BB key for the landmark vector.
/// @param standoff_goal_bb_key  BB key for standoff WaypointGoal (position
///                              norm = standoff distance).
/// @param output_bb_key         BB key to store the computed WaypointGoal.
static std::function<std::string(yasmin::Blackboard::SharedPtr)>
make_standoff_waypoint_cb(const std::string& landmarks_bb_key,
                          const std::string& standoff_goal_bb_key,
                          const std::string& output_bb_key) {
    return [=](yasmin::Blackboard::SharedPtr bb) -> std::string {
        auto landmarks =
            bb->get<std::vector<vortex_msgs::msg::Landmark>>(landmarks_bb_key);

        if (landmarks.empty()) {
            YASMIN_LOG_ERROR("No landmarks found for waypoint computation");
            return ABORT;
        }

        auto landmark_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
            landmarks.front().pose.pose);
        auto standoff_goal = bb->get<WaypointGoal>(standoff_goal_bb_key);

        const Eigen::Vector3d p_valve = landmark_pose.pos_vector();
        const Eigen::Quaterniond q_valve =
            landmark_pose.ori_quaternion().normalized();

        // Standoff position: offset along the valve's outward z-normal in odom.
        const Eigen::Vector3d standoff_odom =
            q_valve * standoff_goal.pose.pos_vector();
        const Eigen::Vector3d p_standoff = p_valve + standoff_odom;

        // Extract handle yaw: z-rotation of the valve quaternion in Euler.
        // 90 deg = handle straight reference position.
        // Use this to set the drone heading so it faces the valve handle.
        const double yaw = vortex::utils::math::quat_to_euler(q_valve).z();
        const Eigen::Quaterniond q_drone =
            vortex::utils::math::euler_to_quat(0.0, 0.0, yaw + M_PI);

        WaypointGoal computed_goal;
        computed_goal.pose = Pose::from_eigen(p_standoff, q_drone);
        computed_goal.mode = standoff_goal.mode;
        computed_goal.convergence_threshold =
            standoff_goal.convergence_threshold;

        bb->set<WaypointGoal>(output_bb_key, computed_goal);

        YASMIN_LOG_INFO(
            "Standoff waypoint: pos=[%.3f, %.3f, %.3f], "
            "handle_yaw=%.1f deg",
            p_standoff.x(), p_standoff.y(), p_standoff.z(), yaw * 180.0 / M_PI);

        return SUCCEED;
    };
}

/// @brief Compute a convergence waypoint using TCP kinematic inversion so that
/// the gripper tip lands exactly on the valve handle center.
///
/// Drone orientation is derived from the handle yaw (yaw + 180° to face
/// the valve). Gripper roll is stored on the blackboard for future use.
///
/// p_base = p_valve - R_drone * tcp_offset
///
/// @param landmarks_bb_key BB key for the landmark vector.
/// @param output_bb_key    BB key to store the computed WaypointGoal.
static std::function<std::string(yasmin::Blackboard::SharedPtr)>
make_convergence_waypoint_cb(const std::string& landmarks_bb_key,
                             const std::string& output_bb_key) {
    return [=](yasmin::Blackboard::SharedPtr bb) -> std::string {
        auto landmarks =
            bb->get<std::vector<vortex_msgs::msg::Landmark>>(landmarks_bb_key);

        if (landmarks.empty()) {
            YASMIN_LOG_ERROR("No landmarks found for waypoint computation");
            return ABORT;
        }

        auto landmark_pose = vortex::utils::ros_conversions::ros_pose_to_pose(
            landmarks.front().pose.pose);
        auto tcp_goal = bb->get<WaypointGoal>("tcp_offset_goal");

        const Eigen::Vector3d p_valve = landmark_pose.pos_vector();
        const Eigen::Quaterniond q_valve =
            landmark_pose.ori_quaternion().normalized();

        // Extract handle yaw: z-rotation of the valve quaternion in Euler.
        // 90 deg = handle straight reference position.
        const double yaw = vortex::utils::math::quat_to_euler(q_valve).z();

        // Drone faces back at the valve: heading = yaw + 180°.
        const Eigen::Quaterniond q_drone =
            vortex::utils::math::euler_to_quat(0.0, 0.0, yaw + M_PI);

        // TCP kinematic inversion:
        // p_tcp = p_base + R_drone * tcp_offset  =>  p_base = p_valve - R_drone
        // * tcp_offset
        const Eigen::Vector3d tcp_body = tcp_goal.pose.pos_vector();
        const Eigen::Vector3d p_base = p_valve - q_drone * tcp_body;

        // Store gripper roll for later use (servo command, not yet applied).
        const double gripper_roll = compute_gripper_roll(q_drone, q_valve);
        bb->set<double>("gripper_roll_angle", gripper_roll);
        bb->set<Eigen::Quaterniond>("valve_quaternion", q_valve);

        WaypointGoal computed_goal;
        computed_goal.pose = Pose::from_eigen(p_base, q_drone);
        computed_goal.mode = tcp_goal.mode;
        computed_goal.convergence_threshold = tcp_goal.convergence_threshold;

        bb->set<WaypointGoal>(output_bb_key, computed_goal);

        YASMIN_LOG_INFO(
            "TCP waypoint: base=[%.3f, %.3f, %.3f], "
            "handle_yaw=%.1f deg, gripper_roll=%.1f deg",
            p_base.x(), p_base.y(), p_base.z(), yaw * 180.0 / M_PI,
            gripper_roll * 180.0 / M_PI);

        return SUCCEED;
    };
}

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    vortex_msgs::msg::LandmarkType landmark_type;
    vortex_msgs::msg::LandmarkSubtype landmark_subtype;

    landmark_type.value = vortex_msgs::msg::LandmarkType::VALVE;

    if (config.vertical_mounted_valve) {
        landmark_subtype.value =
            vortex_msgs::msg::LandmarkSubtype::VALVE_VERTICAL;
    } else {
        landmark_subtype.value =
            vortex_msgs::msg::LandmarkSubtype::VALVE_HORIZONTAL;
    }

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    // --- WAIT_FOR_START ---
    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<ServiceTriggerWaitState>(config.start_mission_service),
        {{SUCCEED, "POLL_LANDMARK"}, {CANCEL, ABORT}});

    // --- POLL_LANDMARK: initial landmark poll ---
    sm->add_state("POLL_LANDMARK",
                  std::make_shared<LandmarkPollingState>(
                      config.landmark_polling_action_server, landmark_type,
                      landmark_subtype, "landmarks", "landmark_found"),
                  {{"landmark_found", "COMPUTE_APPROACH"}, {ABORT, ABORT}});

    // --- COMPUTE_APPROACH: 1m from valve along handle yaw direction, facing it
    // ---
    sm->add_state(
        "COMPUTE_APPROACH",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED, ABORT},
            make_standoff_waypoint_cb("landmarks", "standoff_waypoint_goal",
                                      "computed_approach_goal")),
        {{SUCCEED, "APPROACH_WAYPOINT"}, {ABORT, ABORT}});

    // --- APPROACH_WAYPOINT: navigate to approach position ---
    sm->add_state(
        "APPROACH_WAYPOINT",
        std::make_shared<BlackboardWaypointState>(
            config.waypoint_manager_action_server, "computed_approach_goal"),
        {{SUCCEED, "POLL_LANDMARK_2"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // --- POLL_LANDMARK_2: re-poll for refined valve estimate ---
    sm->add_state("POLL_LANDMARK_2",
                  std::make_shared<LandmarkPollingState>(
                      config.landmark_polling_action_server, landmark_type,
                      landmark_subtype, "landmarks", "landmark_found"),
                  {{"landmark_found", "COMPUTE_CONVERGENCE"}, {ABORT, ABORT}});

    // --- COMPUTE_CONVERGENCE: gripper tip onto valve via TCP inversion ---
    sm->add_state("COMPUTE_CONVERGENCE",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{SUCCEED, ABORT},
                      make_convergence_waypoint_cb(
                          "landmarks", "computed_convergence_goal")),
                  {{SUCCEED, "CONVERGE_WAYPOINT"}, {ABORT, ABORT}});

    // --- CONVERGE_WAYPOINT: navigate gripper tip onto valve ---
    sm->add_state(
        "CONVERGE_WAYPOINT",
        std::make_shared<BlackboardWaypointState>(
            config.waypoint_manager_action_server, "computed_convergence_goal"),
        {{SUCCEED, "COMPUTE_RETREAT"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // --- COMPUTE_RETREAT: backs off 1m along handle yaw direction (same as
    // approach) ---
    sm->add_state(
        "COMPUTE_RETREAT",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED, ABORT},
            make_standoff_waypoint_cb("landmarks", "standoff_waypoint_goal",
                                      "computed_retreat_goal")),
        {{SUCCEED, "RETREAT_WAYPOINT"}, {ABORT, ABORT}});

    // --- RETREAT_WAYPOINT: back away from valve ---
    sm->add_state(
        "RETREAT_WAYPOINT",
        std::make_shared<BlackboardWaypointState>(
            config.waypoint_manager_action_server, "computed_retreat_goal"),
        {{SUCCEED, "DONE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // --- DONE ---
    sm->add_state(
        "DONE",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED},
            [](auto) {
                YASMIN_LOG_INFO("Visual inspection mission completed");
                return SUCCEED;
            }),
        {{SUCCEED, SUCCEED}});

    return sm;
}
