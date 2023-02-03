
#include <opencv2/aruco.hpp>

#include <ros/ros.h>



ArucoDetectionNode::ArucoDetectionNode ():loop_rate(10) {
    op_sub = node.subscribe("/zed2/zed_node/rgb/image_rect_color",10, &ArucoDetectionNode::callback, this);

    op_pub = node.advertise<vortex_msgs::ObjectPosition>("object_positions_out",10);
}

void ArucoDetectionNode::callback(sensor_msgs::Image::ConstPtr& img){
    
    objectPositions[objPos.objectID] = objPos.position;
    op_pub.publish(objPos);            
}

void ArucoDetectionNode::execute(){
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ArucoDetectionNode::printMap(std::map<std::string,geometry_msgs::Point> objectsMap){
    for(auto elem : objectsMap){
        ROS_INFO("ID: %s", elem.first.c_str());
        ROS_INFO("position: %f,%f,%f",elem.second.x,elem.second.y,elem.second.z);
            
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"ArucoDetectionNode");
    ArucoDetectionNode arucoNode;
    arucoNode.execute();
}