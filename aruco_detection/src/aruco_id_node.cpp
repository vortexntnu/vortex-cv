#include "aruco_id_node.hpp"

ArucoIdNode::ArucoIdNode() 
: loop_rate{10}
{
    dictionary = new cv::aruco::Dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
}

void ArucoIdNode::callback(const sensor_msgs::ImageConstPtr& img_source)
{
    const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cvImage->image;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    cv::aruco::detectMarkers(img, dictionary, corners, ids);

    if (ids.size() == 0) return;

    

    cv::aruco::drawDetectedMarkers(img, corners, ids);
    publishCVImg(img, cvImage->header.stamp);
}

void ArucoIdNode::publishCVImg(const cv::Mat& img, ros::Time timestamp)
{
    static size_t counter{0};
    cv_bridge::CvImage imgBridge;
    sensor_msgs::Image imgMsg; 

    std_msgs::Header header;
    header.seq = counter++; 
    header.stamp = timestamp; // Should the time now be used, or the time the image was taken be used?
    imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    imgBridge.toImageMsg(imgMsg); 
    opImagePub.publish(imgMsg);
}



void ArucoIdNode::execute()
{
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }
}





int main(int argc, char **argv) {
    ros::init(argc,argv,"ArucoIdNode");

    ArucoIdNode arucoNode;
    arucoNode.execute();
}