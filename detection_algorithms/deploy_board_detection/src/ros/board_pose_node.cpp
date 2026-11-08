#include "ros/board_pose_node.hpp"
#include "lib/depth_to_points.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


#include <rclcpp/rclcpp.hpp>
#include <vector>


namespace vortex::board_detection {

BoardDetectionNode::BoardDetectionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("board_detection_node", options) {
    declare_parameters();
    setup_publishers_and_subscribers();
}

void BoardDetectionNode::declare_parameters() {
    //oblig subs
    this->declare_parameter<std::string>("topic.board_1_sub_topic");
    this->declare_parameter<std::string>("topic.board_2_sub_topic");
    this->declare_parameter<std::string>("topic.depth_image_sub_topic");
    this->declare_parameter<std::string>("topic.drone_pose_sub_topic");

    //oblig pubs
    this->declare_parameter<std::string>("topic.board_pose_pub_topic");
    this->declare_parameter<std::string>("topic.small_search_rescue_pub_topic");
    this->declare_parameter<std::string>("topic.large_search_rescue_pub_topic");
    this->declare_parameter<std::string>("topic.small_survey_repair_pub_topic");
    this->declare_parameter<std::string>("topic.large_survey_repair_pub_topic");

    //camera
    horizontal_fov_ =
        this->declare_parameter<double>("camera.horizontal_fov");

    vertical_fov_ =
        this->declare_parameter<double>("camera.vertical_fov");

    //RANSAC
    distance_threshold_ =
        this->declare_parameter<double>("ransac.distance_threshold");

    num_iterations_ =
        this->declare_parameter<int>("ransac.num_iterations");

    min_inliers_ =
        this->declare_parameter<int>("ransac.min_inliers");
}

void BoardDetectionNode::setup_publishers_and_subscribers() {
    const auto qos_profile =
        vortex::utils::qos_profiles::sensor_data_profile(1);

    //oblig subs
    const std::string board_1_sub_topic =
        this->get_parameter("topic.board_1_sub_topic").as_string();
    const std::string board_2_sub_topic =
        this->get_parameter("topic.board_2_sub_topic").as_string();
    const std::string depth_image_sub_topic =
        this->get_parameter("topic.depth_image_sub_topic").as_string();
    const std::string drone_pose_sub_topic =
        this->get_parameter("topic.drone_pose_sub_topic").as_string();

    board_1_sub_.subscribe(this, board_1_sub_topic, qos_profile);
    board_2_sub_.subscribe(this, board_2_sub_topic, qos_profile);
    depth_image_sub_.subscribe(this, depth_image_sub_topic, qos_profile);
    drone_pose_sub_.subscribe(this, drone_pose_sub_topic, qos_profile);

    //Filtrerer for hvert av brettene
    board_1_sync_ =
        std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10),
            board_1_sub_,
            depth_image_sub_,
            drone_pose_sub_
    );

    board_2_sync_ =
        std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10),
            board_2_sub_,
            depth_image_sub_,
            drone_pose_sub_
    ); 

    board_1_sync_->registerCallback(
        std::bind(
            &BoardDetectionNode::board_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            1
            )
        );

    board_2_sync_->registerCallback(
        std::bind(
            &BoardDetectionNode::board_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            2
            )
        );
    

    //oblig pubs
    const std::string board_pose_pub_topic =
        this->get_parameter("topic.board_pose_pub_topic").as_string();
    const std::string small_search_rescue_pub_topic =
        this->get_parameter("topic.small_search_rescue_pub_topic").as_string();
    const std::string large_search_rescue_pub_topic =
        this->get_parameter("topic.large_search_rescue_pub_topic").as_string();
    const std::string small_survey_repair_pub_topic =
        this->get_parameter("topic.small_survey_repair_pub_topic").as_string();
    const std::string large_survey_repair_pub_topic =
        this->get_parameter("topic.large_survey_repair_pub_topic").as_string();

    board_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        board_pose_pub_topic, qos_profile);
    small_search_rescue_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        small_search_rescue_pub_topic, qos_profile);
    large_search_rescue_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        large_search_rescue_pub_topic, qos_profile);
    small_survey_repair_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        small_survey_repair_pub_topic, qos_profile);
    large_survey_repair_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        large_survey_repair_pub_topic, qos_profile);

    //debug subs

    //debug pubs
}

void BoardDetectionNode::board_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& board_mask,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& drone_pose,
    int board_type
) { 
//First off: Run checks (if img is treatable etc.)
//Step 1: Convert ROS image message to OpenCV image
    cv::Mat board_mask_mat = cv_bridge::toCvCopy(board_mask, "mono8")->image;
    cv::Mat depth_image_mat = cv_bridge::toCvCopy(depth_image, "32FC1")->image;

//Step 2: Turn stereo-image to point cloud
    std::vector<geometry_msgs::msg::Point32> relative_board_point_cloud =
        depthToPoints(
            depth_image_mat,
            board_mask_mat,
            horizontal_fov_,
            vertical_fov_
        );

//Step 3: RANSAC to find the board plane
    Plane relative_plane =
        ransacPlaneFitting(
            relative_board_point_cloud,
            distance_threshold_,
            num_iterations_,
            min_inliers_
        ); //TODO

//Step 4: Use plane + mask to find board center (+ corners)
    geometry_msgs::msg::Pose relative_board_pose = 
        findBoardPose(
            relative_plane, 
            board_mask_mat,
            horizontal_fov_,
            vertical_fov_
        ); //TODO

//Step 5: Transfer board pose to world frame
    geometry_msgs::msg::PoseStamped world_board_pose = 
        transformBoardPoseToWorldFrame(
            relative_board_pose, 
            *drone_pose
        ); //TODO

//Step 6: publish board pose
    board_pose_pub_->publish(world_board_pose);
//Step 7: Start circle_callback???
}

struct Plane {
    double a;
    double b;
    double c;
    double d;
};

}