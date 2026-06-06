#include "bearing_direction_server/bearing_direction_base.hpp"

#include <spdlog/spdlog.h>
#include <tf2/exceptions.h>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace bearing_direction_server {

BearingDirectionBase::BearingDirectionBase(const std::string& node_name,
                                           const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options) {
    declare_base_parameters();
    target_frame_ = get_parameter("target_frame").as_string();
    outlier_threshold_deg_ = get_parameter("outlier_threshold_deg").as_double();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BearingDirectionBase::~BearingDirectionBase() {
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
}

void BearingDirectionBase::declare_base_parameters() {
    declare_parameter<std::string>("target_frame", "nautilus/odom");
    declare_parameter<std::string>("topics.odom", "/nautilus/odom");
    declare_parameter<std::string>("action_name", "collect_bearing_direction");
    declare_parameter<double>("outlier_threshold_deg", 30.0);
}

void BearingDirectionBase::setup_base() {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("topics.odom").as_string(),
        vortex::utils::qos_profiles::sensor_data_profile(1),
        std::bind(&BearingDirectionBase::odom_callback, this,
                  std::placeholders::_1));

    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/bearing_direction_markers", rclcpp::QoS(1).transient_local());

    const std::string action_name = get_parameter("action_name").as_string();
    action_server_ = rclcpp_action::create_server<Action>(
        this, action_name,
        [this](auto uuid, auto goal) { return handle_goal(uuid, goal); },
        [this](auto handle) { return handle_cancel(handle); },
        [this](auto handle) { return handle_accepted(handle); });

    spdlog::info("BearingDirectionBase '{}' ready. action='{}', frame='{}'",
                 get_name(), action_name, target_frame_);
}

// ---------------------------------------------------------------------------

void BearingDirectionBase::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard lock(mutex_);
    latest_drone_pos_ = msg->pose.pose.position;
}

void BearingDirectionBase::add_direction(const Eigen::Vector3d& dir_odom) {
    std::lock_guard lock(mutex_);
    if (!collecting_) return;
    accumulated_dirs_.push_back(dir_odom);
}

std::optional<Eigen::Vector3d> BearingDirectionBase::rotate_to_odom(
    const Eigen::Vector3d& dir, const std::string& src_frame,
    const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            target_frame_, src_frame, stamp,
            rclcpp::Duration::from_seconds(0.2));
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("BearingDirectionBase '{}': TF ({} -> {}): {}",
                     get_name(), src_frame, target_frame_, ex.what());
        return std::nullopt;
    }
    const auto& rot = tf_stamped.transform.rotation;
    const Eigen::Quaterniond q(rot.w, rot.x, rot.y, rot.z);
    return (q * dir).normalized();
}

// ---------------------------------------------------------------------------
// Action server
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse BearingDirectionBase::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const Action::Goal> goal) {
    if (goal->timeout_sec <= 0.0) {
        spdlog::warn("BearingDirectionBase '{}': invalid timeout {:.2f}s.",
                     get_name(), goal->timeout_sec);
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->min_measurements < 1) {
        spdlog::warn("BearingDirectionBase '{}': min_measurements must be >= 1.",
                     get_name());
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->max_measurements > 0 &&
        goal->max_measurements < goal->min_measurements) {
        spdlog::warn(
            "BearingDirectionBase '{}': max_measurements ({}) < min_measurements ({}).",
            get_name(), goal->max_measurements, goal->min_measurements);
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BearingDirectionBase::handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/) {
    spdlog::info("BearingDirectionBase '{}': cancel requested.", get_name());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BearingDirectionBase::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    preempted_ = false;
    execute_thread_ = std::thread([this, goal_handle]() { execute(goal_handle); });
}

void BearingDirectionBase::execute(
    const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Action::Feedback>();
    auto result = std::make_shared<Action::Result>();

    {
        std::lock_guard lock(mutex_);
        accumulated_dirs_.clear();
        collecting_ = true;
    }

    spdlog::info(
        "BearingDirectionBase '{}': collecting for {:.1f}s, dist={:.1f}m, "
        "min={} max={}.",
        get_name(), goal->timeout_sec, goal->distance,
        goal->min_measurements,
        goal->max_measurements > 0 ? std::to_string(goal->max_measurements) : "unlimited");

    const rclcpp::Time start = now();
    rclcpp::Rate rate(10.0);

    while (rclcpp::ok()) {
        if (preempted_.load()) {
            std::lock_guard lock(mutex_);
            collecting_ = false;
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        if (goal_handle->is_canceling()) {
            std::lock_guard lock(mutex_);
            collecting_ = false;
            result->success = false;
            goal_handle->canceled(result);
            spdlog::info("BearingDirectionBase '{}': canceled.", get_name());
            return;
        }

        const double remaining = goal->timeout_sec - (now() - start).seconds();
        if (remaining <= 0.0) break;

        {
            std::lock_guard lock(mutex_);
            feedback->measurements_collected =
                static_cast<int32_t>(accumulated_dirs_.size());
            if (goal->max_measurements > 0 &&
                feedback->measurements_collected >= goal->max_measurements) {
                collecting_ = false;
                break;
            }
        }
        {
            std::lock_guard lock(mutex_);
            publish_viz_markers(accumulated_dirs_, latest_drone_pos_,
                                std::nullopt, goal->distance);
        }
        feedback->time_remaining_sec = remaining;
        goal_handle->publish_feedback(feedback);
        rate.sleep();
    }

    // Snapshot state
    std::vector<Eigen::Vector3d> dirs;
    std::optional<geometry_msgs::msg::Point> drone_pos;
    {
        std::lock_guard lock(mutex_);
        collecting_ = false;
        dirs = accumulated_dirs_;
        drone_pos = latest_drone_pos_;
    }

    if (dirs.empty()) {
        spdlog::warn("BearingDirectionBase '{}': no measurements collected.",
                     get_name());
        result->success = false;
        goal_handle->succeed(result);
        return;
    }

    if (static_cast<int32_t>(dirs.size()) < goal->min_measurements) {
        spdlog::warn(
            "BearingDirectionBase '{}': only {} measurements collected, need {}.",
            get_name(), dirs.size(), goal->min_measurements);
        result->success = false;
        goal_handle->succeed(result);
        return;
    }

    const Eigen::Vector3d avg_dir =
        filtered_mean(dirs, outlier_threshold_deg_);

    // Build pose: position = drone_pos + avg_dir * distance
    const double px = drone_pos ? drone_pos->x : 0.0;
    const double py = drone_pos ? drone_pos->y : 0.0;
    const double pz = drone_pos ? drone_pos->z : 0.0;

    if (!drone_pos) {
        spdlog::warn("BearingDirectionBase '{}': no odom yet, using origin.",
                     get_name());
    }

    result->pose.position.x = px + avg_dir.x() * goal->distance;
    result->pose.position.y = py + avg_dir.y() * goal->distance;
    result->pose.position.z = pz + avg_dir.z() * goal->distance;

    // Orientation: yaw towards the target direction
    const double yaw = std::atan2(avg_dir.y(), avg_dir.x());
    result->pose.orientation.z = std::sin(yaw * 0.5);
    result->pose.orientation.w = std::cos(yaw * 0.5);

    result->success = true;

    publish_viz_markers(dirs, drone_pos, avg_dir, goal->distance);

    spdlog::info(
        "BearingDirectionBase '{}': {} measurements ({} after filtering), "
        "avg_dir=[{:.3f},{:.3f},{:.3f}], waypoint=[{:.2f},{:.2f},{:.2f}]",
        get_name(), dirs.size(),
        static_cast<int>(dirs.size()),  // filtered count logged separately
        avg_dir.x(), avg_dir.y(), avg_dir.z(),
        result->pose.position.x, result->pose.position.y,
        result->pose.position.z);

    goal_handle->succeed(result);
}

// ---------------------------------------------------------------------------

Eigen::Vector3d BearingDirectionBase::filtered_mean(
    const std::vector<Eigen::Vector3d>& dirs, double threshold_deg) {
    // Spherical mean
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& d : dirs) sum += d;
    if (sum.norm() < 1e-6) return Eigen::Vector3d::UnitX();
    const Eigen::Vector3d mean = sum.normalized();

    // One pass: keep vectors within threshold_deg of mean
    const double cos_thresh = std::cos(threshold_deg * M_PI / 180.0);
    Eigen::Vector3d filtered_sum = Eigen::Vector3d::Zero();
    int kept = 0;
    for (const auto& d : dirs) {
        if (d.dot(mean) >= cos_thresh) {
            filtered_sum += d;
            ++kept;
        }
    }

    if (kept == 0) {
        spdlog::warn(
            "BearingDirectionBase: all {} vectors rejected as outliers, "
            "falling back to unfiltered mean.",
            dirs.size());
        return mean;
    }

    spdlog::debug("BearingDirectionBase: kept {}/{} vectors after outlier rejection.",
                  kept, dirs.size());
    return filtered_sum.normalized();
}

void BearingDirectionBase::publish_viz_markers(
    const std::vector<Eigen::Vector3d>& dirs,
    const std::optional<geometry_msgs::msg::Point>& drone_pos,
    const std::optional<Eigen::Vector3d>& final_dir,
    double distance) {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(del);

    const double ox = drone_pos ? drone_pos->x : 0.0;
    const double oy = drone_pos ? drone_pos->y : 0.0;
    const double oz = drone_pos ? drone_pos->z : 0.0;
    const rclcpp::Time stamp = now();

    // Individual bearing ray arrows (blue)
    int id = 0;
    constexpr double ray_len = 3.0;
    for (const auto& d : dirs) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp = stamp;
        arrow.header.frame_id = target_frame_;
        arrow.ns = "bearing_rays";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.04;
        arrow.scale.y = 0.08;
        arrow.scale.z = 0.08;
        arrow.color.r = 0.2f;
        arrow.color.g = 0.6f;
        arrow.color.b = 1.0f;
        arrow.color.a = 0.6f;
        arrow.lifetime = rclcpp::Duration::from_seconds(4.0);
        geometry_msgs::msg::Point start, end;
        start.x = ox; start.y = oy; start.z = oz;
        end.x = ox + d.x() * ray_len;
        end.y = oy + d.y() * ray_len;
        end.z = oz + d.z() * ray_len;
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        array.markers.push_back(arrow);
    }

    // Final averaged direction arrow (orange) + target sphere
    if (final_dir.has_value()) {
        const Eigen::Vector3d& fd = *final_dir;

        visualization_msgs::msg::Marker avg_arrow;
        avg_arrow.header.stamp = stamp;
        avg_arrow.header.frame_id = target_frame_;
        avg_arrow.ns = "bearing_avg_direction";
        avg_arrow.id = 0;
        avg_arrow.type = visualization_msgs::msg::Marker::ARROW;
        avg_arrow.action = visualization_msgs::msg::Marker::ADD;
        avg_arrow.scale.x = 0.08;
        avg_arrow.scale.y = 0.16;
        avg_arrow.scale.z = 0.16;
        avg_arrow.color.r = 1.0f;
        avg_arrow.color.g = 0.5f;
        avg_arrow.color.b = 0.0f;
        avg_arrow.color.a = 1.0f;
        avg_arrow.lifetime = rclcpp::Duration::from_seconds(0.0);  // persist
        geometry_msgs::msg::Point start, end;
        start.x = ox; start.y = oy; start.z = oz;
        end.x = ox + fd.x() * distance;
        end.y = oy + fd.y() * distance;
        end.z = oz + fd.z() * distance;
        avg_arrow.points.push_back(start);
        avg_arrow.points.push_back(end);
        array.markers.push_back(avg_arrow);

        visualization_msgs::msg::Marker sphere;
        sphere.header.stamp = stamp;
        sphere.header.frame_id = target_frame_;
        sphere.ns = "bearing_target";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = end.x;
        sphere.pose.position.y = end.y;
        sphere.pose.position.z = end.z;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.3;
        sphere.scale.y = 0.3;
        sphere.scale.z = 0.3;
        sphere.color.r = 1.0f;
        sphere.color.g = 0.35f;
        sphere.color.b = 0.0f;
        sphere.color.a = 1.0f;
        sphere.lifetime = rclcpp::Duration::from_seconds(0.0);  // persist
        array.markers.push_back(sphere);
    }

    viz_pub_->publish(array);
}

}  // namespace bearing_direction_server
