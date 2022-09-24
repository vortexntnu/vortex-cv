#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "image_preprocessing_cpp/ImagePreprocessing.hpp"


class ImagePreprocessingNode
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    

private:
    std::string image_topic{"/zed2/zed_node/rgb/image_rect_color"};
    std::string clahe_topic{"/image_preprocessing/clahe"};
    bool useCLAHE{true};
    double clahe_clip_limit{2};
    double clahe_tile_grid_size{8};
    int clahe_plane{0};

public:
    ImagePreprocessingNode()
        : it_(nh_)
        {
            nh_.getParam("useCLAHE", useCLAHE);
            nh_.getParam("imageTopic", image_topic);
            nh_.getParam("claheTopic", clahe_topic);
            nh_.getParam("claheClipLimit", clahe_clip_limit);
            nh_.getParam("claheTileGridSize", clahe_tile_grid_size);
            nh_.getParam("clahePlane", clahe_plane);

            image_sub_ = it_.subscribe(image_topic, 1, &ImagePreprocessingNode::image_callback, this);
            image_pub_ = it_.advertise(clahe_topic, 1);
        }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        ImagePreprocessing image_preprocessing_(clahe_clip_limit, clahe_tile_grid_size);
        std_msgs::Header header = msg->header;
        cv::Mat cv_image_;
        try
        {
            cv_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if(useCLAHE){
            cv::Mat clahe_image_ = image_preprocessing_.doClahe(cv_image_, clahe_plane);
            image_pub_.publish(cv_bridge::CvImage(header, "bgr8", clahe_image_).toImageMsg());

        }
    }

    void image_publish(cv_bridge::CvImagePtr cv_ptr)
    {
        image_pub_.publish(cv_ptr -> toImageMsg());
    }
    
};

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "image_preprocessing_cpp_node");
    ros::NodeHandle nh;
    ImagePreprocessingNode wrapper;
    ros::spin();

    return 0;
}