#include <ros/ros.h>
#include <sensor_msgs/Image.h> // Read and publish images
#include <cv_bridge/cv_bridge.h>


#include <image_processing.hpp>
#include <filter_params_rqt.hpp>

class ImageFilteringNode {
public:
    ImageFilteringNode() : loop_rate{10}, filterParams{}
    {
	    // opImageSub = node.subscribe("/zed2/zed_node/left/image_rect_color", 10, &ImageFilteringNode::callback, this);
	    opImageSub = node.subscribe("/udfc/wrapper/camera_raw", 10, &ImageFilteringNode::callback, this);
	    opImagePub = node.advertise<sensor_msgs::Image>("filtered_image", 100);
    }
    /**
	 * The callback function for the op_sub-subscriber.
	 */
	void callback(const sensor_msgs::ImageConstPtr &img_source)
    {
        // Input
        const cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(img_source, sensor_msgs::image_encodings::BGR8);

        cv::Mat img = cvImage->image;
        if (img.empty()) 
        {
            ROS_INFO_STREAM("DOCKING_NODE: Empty image");
            return;
        }
        cv::Mat filteredImg;
        filter_from_rqt(img, filteredImg, filterParams.configs);
	    publishCVImg(filteredImg, cvImage->header.stamp);
    }

	/**
	 * ros::spinOnce() is called at 10Hz
	 */
	void execute()
    {
	while (ros::ok()) 
    {
		ros::spinOnce();
		loop_rate.sleep();
	}
    }

    void publishCVImg(const cv::Mat &img, ros::Time timestamp = ros::Time::now())
    {
        static size_t counter{0};
        cv_bridge::CvImage imgBridge;
        sensor_msgs::Image imgMsg;

        std_msgs::Header header;
        header.seq   = counter++;
        header.stamp = timestamp;
        imgBridge    = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
        imgBridge.toImageMsg(imgMsg);
        opImagePub.publish(imgMsg);
    }
protected:
	// rqt stuff
	FilterParams_rqt filterParams;
    // ROS stuff
	ros::NodeHandle node;
	ros::Rate loop_rate;
	ros::Subscriber opImageSub;
	ros::Publisher opImagePub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ImageFilteringNode");

	ImageFilteringNode filteringNode;

	filteringNode.execute();
}