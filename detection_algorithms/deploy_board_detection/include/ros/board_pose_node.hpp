#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class BoardDetectionNode : public rclcpp::Node {
    public:
        explicit BoardDetectionNode(const rclcpp::NodeOptions& options);

        ~BoardDetectionNode() override = default;
    
        private:
            // Input subscriptions
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_camera_sub_;


            // Result publishers
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr board_pose_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr small_search_rescue_pub_;   
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr large_search_rescue_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr small_survey_repair_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr large_survey_repair_pub_;

            // Debug subscriptions
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colour_camera_sub_;

            // Debug publishers
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr contour_pub_;

            void declare_parameters();

            void setup_publishers_and_subscribers();

            void image_callback(); // Define input values, send into other functions

};
