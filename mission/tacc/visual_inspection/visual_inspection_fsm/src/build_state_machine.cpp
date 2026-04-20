#include "visual_inspection_fsm/states.hpp"

#include <chrono>
#include <thread>

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/service_trigger_wait_state.hpp>

using vortex::utils::types::Pose;
using vortex::utils::types::WaypointMode;
using vortex::utils::waypoints::WaypointGoal;
using vortex_yasmin_utils::LandmarkPollingState;
using vortex_yasmin_utils::ServiceTriggerWaitState;
using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

using WaypointManagerAction = vortex_msgs::action::WaypointManager;

/// @brief ActionState that reads a WaypointGoal from the blackboard and sends
/// it to the WaypointManager. Used for dynamically computed waypoints
/// where the pose is calculated at runtime.
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

/// @brief Extract the valve's outward direction vector from the valve
/// quaternion. The valve +z axis is always the valve normal in the odom frame;
/// this direction points from the valve surface outward toward the drone side.
static Eigen::Vector3d valve_direction(const Eigen::Quaterniond& q_valve) {
    return (q_valve * Eigen::Vector3d::UnitZ()).normalized();
}

/// @brief Compute the drone base_link orientation such that its +x axis
/// points toward the valve (opposite the valve normal direction) with zero
/// roll. Yaw and pitch are derived from the direction vector so the drone
/// is parallel to the drone-to-valve axis.
static Eigen::Quaterniond compute_drone_orientation(
    const Eigen::Vector3d& valve_dir) {
    // Drone +x should point from drone toward valve = -valve_dir.
    const Eigen::Vector3d d = -valve_dir.normalized();

    const double yaw = std::atan2(d.y(), d.x());
    const double pitch =
        std::atan2(-d.z(), std::sqrt(d.x() * d.x() + d.y() * d.y()));

    const Eigen::Quaterniond q_drone =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

    return q_drone.normalized();
}

/// @brief Compute an approach waypoint such that the gripper tip is 1m from
/// the valve along the valve's outward normal. Position only (orientation
/// ignored by the controller).
///
/// The direction vector comes from the valve quaternion's +z axis (the
/// valve normal). The standoff target is expressed in the gripper-tip (TCP)
/// frame; the base_link target is obtained by TCP kinematic inversion using
/// the same drone orientation that will be commanded later (drone +x facing
/// valve, roll=0).
///
/// @param landmarks_bb_key      BB key for the landmark vector.
/// @param standoff_goal_bb_key  BB key for standoff WaypointGoal (config).
/// @param output_bb_key         BB key to store the computed WaypointGoal.
static std::function<std::string(yasmin::Blackboard::SharedPtr)>
make_approach_waypoint_cb(const std::string& landmarks_bb_key,
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
        auto tcp_goal = bb->get<WaypointGoal>("tcp_offset_goal");

        const Eigen::Quaterniond q_valve =
            landmark_pose.ori_quaternion().normalized();
        const Eigen::Vector3d p_valve = landmark_pose.pos_vector();

        // Direction vector from the valve orientation (valve +z = outward
        // normal). The standoff offset z in the config is the distance along
        // this direction.
        const Eigen::Vector3d dir = valve_direction(q_valve);
        const double standoff_distance = standoff_goal.pose.pos_vector().z();

        // Standoff target in the gripper-tip (TCP) frame.
        const Eigen::Vector3d p_target_tcp = p_valve + standoff_distance * dir;

        // Drone orientation it will hold during approach/convergence so the
        // TCP inversion is consistent.
        const Eigen::Quaterniond q_drone = compute_drone_orientation(dir);

        // TCP kinematic inversion: p_base = p_tcp - R_drone * tcp_offset.
        const Eigen::Vector3d tcp_body = tcp_goal.pose.pos_vector();
        const Eigen::Vector3d p_target_base = p_target_tcp - q_drone * tcp_body;

        WaypointGoal computed_goal;
        computed_goal.pose =
            Pose::from_eigen(p_target_base, Eigen::Quaterniond::Identity());
        computed_goal.mode = WaypointMode::ONLY_POSITION;
        computed_goal.convergence_threshold =
            standoff_goal.convergence_threshold;

        bb->set<WaypointGoal>(output_bb_key, computed_goal);

        YASMIN_LOG_INFO(
            "Approach waypoint (position-only, TCP-inverted): "
            "p_base=[%.3f, %.3f, %.3f], p_tcp=[%.3f, %.3f, %.3f], "
            "dir=[%.3f, %.3f, %.3f], dist=%.2fm",
            p_target_base.x(), p_target_base.y(), p_target_base.z(),
            p_target_tcp.x(), p_target_tcp.y(), p_target_tcp.z(), dir.x(),
            dir.y(), dir.z(), standoff_distance);

        return SUCCEED;
    };
}

/// @brief Compute an orientation-only waypoint aligning the drone with the
/// drone-to-valve direction, with zero roll. Uses the valve quaternion +z
/// axis as the direction vector.
///
/// @param landmarks_bb_key BB key for the landmark vector.
/// @param output_bb_key    BB key to store the computed WaypointGoal.
static std::function<std::string(yasmin::Blackboard::SharedPtr)>
make_orient_waypoint_cb(const std::string& landmarks_bb_key,
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

        const Eigen::Quaterniond q_valve =
            landmark_pose.ori_quaternion().normalized();

        const Eigen::Vector3d dir = valve_direction(q_valve);
        const Eigen::Quaterniond q_drone = compute_drone_orientation(dir);

        WaypointGoal computed_goal;
        computed_goal.pose = Pose::from_eigen(Eigen::Vector3d::Zero(), q_drone);
        computed_goal.mode = WaypointMode::ONLY_ORIENTATION;
        computed_goal.convergence_threshold = 0.1;

        bb->set<WaypointGoal>(output_bb_key, computed_goal);
        bb->set<Eigen::Quaterniond>("drone_orientation", q_drone);

        const auto euler = vortex::utils::math::quat_to_euler(q_drone);
        YASMIN_LOG_INFO(
            "Orient waypoint (orientation-only): RPY=[%.1f, %.1f, %.1f] deg",
            euler.x() * 180.0 / M_PI, euler.y() * 180.0 / M_PI,
            euler.z() * 180.0 / M_PI);

        return SUCCEED;
    };
}

/// @brief Compute a convergence waypoint: gripper tip goes to the valve
/// position using TCP kinematic inversion. Position-only waypoint.
///
/// p_tcp = p_base + R_drone * tcp_offset  =>  p_base = p_valve - R_drone *
/// tcp_offset. The drone orientation used for the inversion is the one
/// commanded in the previous orient step.
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

        const Eigen::Quaterniond q_valve =
            landmark_pose.ori_quaternion().normalized();
        const Eigen::Vector3d p_valve = landmark_pose.pos_vector();

        // Re-derive drone orientation from the updated valve pose so the
        // TCP inversion uses the current commanded orientation.
        const Eigen::Vector3d dir = valve_direction(q_valve);
        const Eigen::Quaterniond q_drone = compute_drone_orientation(dir);

        // TCP kinematic inversion: base_link target such that gripper tip
        // lands on the valve position.
        const Eigen::Vector3d tcp_body = tcp_goal.pose.pos_vector();
        const Eigen::Vector3d p_target_base = p_valve - q_drone * tcp_body;

        WaypointGoal computed_goal;
        computed_goal.pose =
            Pose::from_eigen(p_target_base, Eigen::Quaterniond::Identity());
        computed_goal.mode = WaypointMode::ONLY_POSITION;
        computed_goal.convergence_threshold = tcp_goal.convergence_threshold;

        bb->set<WaypointGoal>(output_bb_key, computed_goal);

        YASMIN_LOG_INFO(
            "Convergence waypoint (position-only): p_base=[%.3f, %.3f, %.3f]",
            p_target_base.x(), p_target_base.y(), p_target_base.z());

        return SUCCEED;
    };
}

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    const auto standoff_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "standoff_goal");
    const auto tcp_offset_goal =
        blackboard->get<vortex::utils::waypoints::WaypointGoal>(
            "tcp_offset_goal");

    vortex_msgs::msg::LandmarkType valve_type;
    valve_type.value = vortex_msgs::msg::LandmarkType::VALVE;

    if (config.vertical_mounted_valve) {
        landmark_subtype.value = 0;
    } else {
        landmark_subtype.value =
            vortex_msgs::msg::LandmarkSubtype::VALVE_HORIZONTAL;
    }

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    sm->add_state(
        "WAIT_FOR_START",
        std::make_shared<vortex_yasmin_utils::ServiceTriggerWaitState>(
            config.start_mission_service),
        {{SUCCEED, "LANDMARK_POLLING"}, {CANCEL, ABORT}});

    // --- POLL_LANDMARK: initial valve pose ---
    sm->add_state("POLL_LANDMARK",
                  std::make_shared<LandmarkPollingState>(
                      config.landmark_polling_action_server, landmark_type,
                      landmark_subtype, "landmarks", "landmark_found"),
                  {{"landmark_found", "COMPUTE_APPROACH"}, {ABORT, ABORT}});

    // --- COMPUTE_APPROACH: position 1m from valve along valve normal ---
    sm->add_state(
        "COMPUTE_APPROACH",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED, ABORT},
            make_approach_waypoint_cb("landmarks", "standoff_waypoint_goal",
                                      "computed_approach_goal")),
        {{SUCCEED, "APPROACH_WAYPOINT"}, {ABORT, ABORT}});

    // --- APPROACH_WAYPOINT: position-only to the 1m standoff ---
    sm->add_state(
        "APPROACH_WAYPOINT",
        std::make_shared<BlackboardWaypointState>(
            config.waypoint_manager_action_server, "computed_approach_goal"),
        {{SUCCEED, "POLL_LANDMARK_2"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // --- POLL_LANDMARK_2: refined valve pose ---
    sm->add_state("POLL_LANDMARK_2",
                  std::make_shared<LandmarkPollingState>(
                      config.landmark_polling_action_server, landmark_type,
                      landmark_subtype, "landmarks", "landmark_found"),
                  {{"landmark_found", "COMPUTE_ORIENT"}, {ABORT, ABORT}});

    // --- COMPUTE_ORIENT: align drone facing valve, roll=0 ---
    sm->add_state(
        "COMPUTE_ORIENT",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED, ABORT},
            make_orient_waypoint_cb("landmarks", "computed_orient_goal")),
        {{SUCCEED, "ORIENT_WAYPOINT"}, {ABORT, ABORT}});

    // --- ORIENT_WAYPOINT: orientation-only alignment ---
    sm->add_state(
        "ORIENT_WAYPOINT",
        std::make_shared<BlackboardWaypointState>(
            config.waypoint_manager_action_server, "computed_orient_goal"),
        {{SUCCEED, "POLL_LANDMARK_3"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // --- POLL_LANDMARK_3: final valve pose before convergence ---
    sm->add_state("POLL_LANDMARK_3",
                  std::make_shared<LandmarkPollingState>(
                      config.landmark_polling_action_server, landmark_type,
                      landmark_subtype, "landmarks", "landmark_found"),
                  {{"landmark_found", "COMPUTE_CONVERGENCE"}, {ABORT, ABORT}});

    // --- COMPUTE_CONVERGENCE: TCP-inverted base_link position ---
    sm->add_state("COMPUTE_CONVERGENCE",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{SUCCEED, ABORT},
                      make_convergence_waypoint_cb(
                          "landmarks", "computed_convergence_goal")),
                  {{SUCCEED, "CONVERGE_WAYPOINT"}, {ABORT, ABORT}});

    // --- CONVERGE_WAYPOINT: position-only, gripper tip onto valve ---
    sm->add_state(
        "CONVERGE_WAYPOINT",
        std::make_shared<BlackboardWaypointState>(
            config.waypoint_manager_action_server, "computed_convergence_goal"),
        {{SUCCEED, "DONE"}, {ABORT, ABORT}, {CANCEL, ABORT}});

    // --- DONE ---
    sm->add_state("DONE",
                  yasmin::CbState::make_shared(
                      yasmin::Outcomes{SUCCEED},
                      [](auto) {
                          YASMIN_LOG_INFO("Valve inspection mission completed");
                          return SUCCEED;
                      }),
                  {{SUCCEED, SUCCEED}});

    return sm;
}
