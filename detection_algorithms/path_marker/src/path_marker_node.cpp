#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class PathMarker2DNode : public rclcpp::Node
{
public:
    PathMarker2DNode() : Node("path_marker_2d_node")
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>("/perception/segmentation_mask", rclcpp::SensorDataQoS(),
    std::bind(&PathMarker2DNode::mask_callback, this, std::placeholders::_1));
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("/nautilus/down_camera_info", rclcpp::SensorDataQoS(),
    std::bind(&PathMarker2DNode::info_callback, this, std::placeholders::_1));

        
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/path_marker/pixel_pose", 10);
        pose3d_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/path_marker/pose", 10);
        debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/path_marker/debug_image", 1);

        RCLCPP_INFO(get_logger(), "path_marker_2d_node started");
    }



private:
    // Lagrer kameraparametrene (intrinsics) fra camera_info
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        ppx_ = msg->k[2];
        ppy_ = msg->k[5];
        camera_frame_ = msg->header.frame_id;
        has_info_ = true;
    }

private:
    void mask_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {

        cv::Mat mask = cv_bridge::toCvShare(msg, "mono8") -> image;

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        if (contours.empty()) {
            RCLCPP_INFO(get_logger(), "No marker found");
            return;
        }
       
        int best_idx = -1;
        double best_area = 0.0;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area > best_area) {
                best_area = area;
                best_idx = static_cast<int>(i);
            }
        }
 
        const double MIN_AREA = 500.0;
        if (best_area < MIN_AREA) {
            RCLCPP_INFO(get_logger(), "Too small area: %.0f px", best_area);            return;
        }

        const std::vector<cv::Point> & marker = contours[best_idx];

        cv::Vec4f line;
        cv::fitLine(marker, line, cv::DIST_L2, 0, 0.01, 0.01);
        float vx = line[0];
        float vy = line[1];
        float cx = line[2];
        float cy = line[3];

        double yaw = std::atan2(vy, vx);
 
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = cx;
        pose.pose.position.y = cy;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        pose_pub_->publish(pose);

        if (has_info_) {
            const double Z = CAMERA_TO_FLOOR;
 
            geometry_msgs::msg::PoseStamped pose3d;
            pose3d.header.stamp = msg->header.stamp;
            pose3d.header.frame_id = camera_frame_;
            pose3d.pose.position.x = (cx - ppx_) * Z / fx_;
            pose3d.pose.position.y = (cy - ppy_) * Z / fy_;
            pose3d.pose.position.z = Z;
            pose3d.pose.orientation = pose.pose.orientation;  // samme vinkel, rotasjon om optisk z-akse
            pose3d_pub_->publish(pose3d);
        }



        const float L = 200.0f;
        cv::Point2f p1(cx - vx * L, cy - vy * L);
        cv::Point2f p2(cx + vx * L, cy + vy * L);

        cv::Mat debug;
        cv::cvtColor(mask, debug, cv::COLOR_GRAY2BGR);

        cv::drawContours(debug, contours, best_idx, cv::Scalar(255, 0, 0), 2);
        cv::line(debug, p1, p2, cv::Scalar(0, 0, 255), 3); 
        cv::circle(debug, cv::Point2f(cx, cy), 8, cv::Scalar(0, 255, 0), -1);
 
        auto out = cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg();
        debug_pub_->publish(*out);

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose3D_pub_;

};




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathMarker2DNode>());
  rclcpp::shutdown();
  return 0;
}

