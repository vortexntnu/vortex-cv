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

/// @brief Compute the drone base_link orientation such that its +x axis
/// points opposite to the valve's +z normal (toward the valve surface),
/// with the remaining axes aligned to the valve handle yaw.
///
/// The handle yaw (z-rotation of the valve quaternion in Euler) is projected
/// onto the plane perpendicular to the approach axis to orient the drone
/// correctly around its approach direction.
static Eigen::Quaterniond compute_drone_orientation(
    const Eigen::Quaterniond& q_valve) {
    // Valve outward normal in odom frame. Drone +x points toward the valve.
    const Eigen::Vector3d n = (q_valve * Eigen::Vector3d::UnitZ()).normalized();
    const Eigen::Vector3d x_drone = -n;

    // Handle yaw: z-rotation of the valve quaternion in Euler.
    // 90 deg = handle straight reference position.
    // Project the handle direction onto the plane perpendicular to x_drone
    // so the drone rotates around its approach axis to align with the handle.
    const double yaw = vortex::utils::math::quat_to_euler(q_valve).z();
    const Eigen::Vector3d handle_dir(std::cos(yaw), std::sin(yaw), 0.0);

    Eigen::Vector3d y_drone = handle_dir - handle_dir.dot(x_drone) * x_drone;

    if (y_drone.norm() < 1e-6) {
        // Degenerate: handle direction is parallel to approach axis.
        // Fall back to world z (down in NED).
        const Eigen::Vector3d world_z(0, 0, 1);
        y_drone = world_z - world_z.dot(x_drone) * x_drone;
    }
    y_drone.normalize();

    const Eigen::Vector3d z_drone = x_drone.cross(y_drone).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x_drone;
    R.col(1) = y_drone;
    R.col(2) = z_drone;

    return Eigen::Quaterniond(R).normalized();
}

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

/// @brief Compute a standoff waypoint: drone base_link goes to
/// p_valve + R_valve * standoff, facing the valve. No TCP inversion.
///
/// @param landmarks_bb_key      BB key for the landmark vector.
/// @param standoff_goal_bb_key  BB key for standoff WaypointGoal.
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

        // --- Valve pose in odom ---
        const Eigen::Quaterniond q_valve =
            landmark_pose.ori_quaternion().normalized();
        const Eigen::Vector3d p_valve = landmark_pose.pos_vector();

        // --- Standoff: base_link goes to valve + offset along normal ---
        const Eigen::Vector3d standoff_odom =
            q_valve * standoff_goal.pose.pos_vector();
        const Eigen::Vector3d p_target = p_valve + standoff_odom;

        // --- Drone orientation (face the valve) ---
        const Eigen::Quaterniond q_drone = compute_drone_orientation(q_valve);

        WaypointGoal computed_goal;
        computed_goal.pose = Pose::from_eigen(p_target, q_drone);
        computed_goal.mode = standoff_goal.mode;
        computed_goal.convergence_threshold =
            standoff_goal.convergence_threshold;

        bb->set<WaypointGoal>(output_bb_key, computed_goal);

        const auto euler = vortex::utils::math::quat_to_euler(q_drone);
        YASMIN_LOG_INFO(
            "Standoff waypoint: base=[%.3f, %.3f, %.3f], "
            "RPY=[%.1f, %.1f, %.1f] deg",
            p_target.x(), p_target.y(), p_target.z(), euler.x() * 180.0 / M_PI,
            euler.y() * 180.0 / M_PI, euler.z() * 180.0 / M_PI);

        return SUCCEED;
    };
}

/// @brief Compute a convergence waypoint: gripper tip goes to the valve
/// position using TCP kinematic inversion. Also stores the valve quaternion
/// and gripper roll angle on the blackboard.
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

        // --- Valve pose in odom (= TCP target) ---
        const Eigen::Quaterniond q_valve =
            landmark_pose.ori_quaternion().normalized();
        const Eigen::Vector3d p_valve = landmark_pose.pos_vector();

        // --- Drone orientation (face the valve) ---
        const Eigen::Quaterniond q_drone = compute_drone_orientation(q_valve);

        // --- TCP kinematic inversion ---
        // p_tcp = p_base + R_drone * tcp_offset
        // => p_base = p_tcp - R_drone * tcp_offset
        const Eigen::Vector3d tcp_body = tcp_goal.pose.pos_vector();
        const Eigen::Vector3d p_target_base = p_valve - q_drone * tcp_body;

        // --- Gripper roll for servo command ---
        const double gripper_roll = compute_gripper_roll(q_drone, q_valve);

        WaypointGoal computed_goal;
        computed_goal.pose = Pose::from_eigen(p_target_base, q_drone);
        computed_goal.mode = tcp_goal.mode;
        computed_goal.convergence_threshold = tcp_goal.convergence_threshold;

        bb->set<WaypointGoal>(output_bb_key, computed_goal);
        bb->set<double>("gripper_roll_angle", gripper_roll);
        bb->set<Eigen::Quaterniond>("valve_quaternion", q_valve);

        const auto euler = vortex::utils::math::quat_to_euler(q_drone);
        YASMIN_LOG_INFO(
            "TCP waypoint: base=[%.3f, %.3f, %.3f], "
            "RPY=[%.1f, %.1f, %.1f] deg, gripper_roll=%.1f deg",
            p_target_base.x(), p_target_base.y(), p_target_base.z(),
            euler.x() * 180.0 / M_PI, euler.y() * 180.0 / M_PI,
            euler.z() * 180.0 / M_PI, gripper_roll * 180.0 / M_PI);

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

    // --- COMPUTE_APPROACH: base_link 0.5m from valve, facing it ---
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

    // --- COMPUTE_RETREAT: base_link backs off 0.5m (same as approach) ---
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
