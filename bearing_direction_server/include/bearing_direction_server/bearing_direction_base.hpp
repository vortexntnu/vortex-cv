#ifndef BEARING_DIRECTION_SERVER__BEARING_DIRECTION_BASE_HPP_
#define BEARING_DIRECTION_SERVER__BEARING_DIRECTION_BASE_HPP_

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vortex_msgs/action/collect_bearing_direction.hpp>

namespace bearing_direction_server {

/**
 * Base class for the three bearing direction action servers.
 *
 * Handles: odom subscription, TF, action server lifecycle, collection loop,
 * outlier-filtered spherical averaging, and pose construction.
 *
 * Subclasses call setup_base() in their constructor after their own
 * parameters are declared, then call add_direction() from their detection
 * callbacks to feed normalised odom-frame direction vectors into the buffer.
 */
class BearingDirectionBase : public rclcpp::Node {
   public:
    using Action = vortex_msgs::action::CollectBearingDirection;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

   protected:
    explicit BearingDirectionBase(const std::string& node_name,
                                  const rclcpp::NodeOptions& options);

    ~BearingDirectionBase();

    // Call from subclass constructor after declaring subclass parameters.
    void setup_base();

    // Subclasses call this for each direction vector already in odom frame,
    // along with the drone position at the time of measurement.
    void add_direction(const Eigen::Vector3d& dir_odom,
                       const Eigen::Vector3d& origin);

    // Called under mutex_ just before collecting_ is set to true.
    // Override in subclasses that maintain per-collection state (e.g. filters).
    virtual void on_collection_start() {}

    // Subclasses that run their own filter can set this to bypass the base
    // class filtered_mean() and use their own final (dir, origin) instead.
    // Cleared at the start of each collection.
    std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        final_result_override_;

    // Rotate a direction vector from src_frame into target_frame_ via TF.
    std::optional<Eigen::Vector3d> rotate_to_odom(const Eigen::Vector3d& dir,
                                                  const std::string& src_frame,
                                                  const rclcpp::Time& stamp);

    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // True while a goal is actively collecting — subclass callbacks check this.
    bool collecting_{false};
    std::mutex mutex_;
    std::optional<geometry_msgs::msg::Point> latest_drone_pos_;

   private:
    void declare_base_parameters();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Action::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    // Spherical mean then one pass of outlier rejection (angle >
    // threshold_deg).
    static Eigen::Vector3d filtered_mean(
        const std::vector<Eigen::Vector3d>& dirs,
        double threshold_deg);

    void publish_viz_markers(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>&
            measurements,
        const std::optional<geometry_msgs::msg::Point>& drone_pos,
        const std::optional<Eigen::Vector3d>& final_dir,
        double distance);

    rclcpp_action::Server<Action>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

    // Each entry is (origin, direction) in odom frame.
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> accumulated_dirs_;
    std::optional<geometry_msgs::msg::Point> collection_start_pos_;
    double outlier_threshold_deg_{30.0};

    std::atomic<bool> preempted_{false};
    std::mutex execute_mutex_;
    std::thread execute_thread_;
};

}  // namespace bearing_direction_server

#endif  // BEARING_DIRECTION_SERVER__BEARING_DIRECTION_BASE_HPP_
