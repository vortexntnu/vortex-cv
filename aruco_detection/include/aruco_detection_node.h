#include <ros/ros.h>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/Image.h>

class ArucoDetectionNode{
    /**
     * To serve as an interface between the perception system and the control system,
     * an instance of this class receives object positions ("landmarks") published by 
     * the perception system on a ROS-topic. The object positions are stored in a map
     * before they are published on a ROS-topic which the control system is subscribed to. 
     */
public:
    ArucoDetectionNode();
    /**
     * The callback function for the op_sub-subscriber.
     * @param objPos is the message received on the ROS-topic, containing an object ID and the position of the object.
     * The object ID and position is stored in the objectPositions-map
     * The message received is further published on the object_positions_out-topic.
     */
void callback(const sensor_msgs::ImageConstPtr& img_source);

    /**
     * ros::spinOnce() is called at 10Hz
     */
    void execute();


protected:
    ros::NodeHandle node;
    ros::Subscriber op_sub;
    ros::Publisher op_pub;
    ros::Rate loop_rate;

    // Dictionary of accepted ArUco marker-IDs
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    // Parameters for camera calibration
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
};