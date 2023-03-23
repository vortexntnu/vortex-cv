#include "aruco_filtering_node.hpp"

ArucoFilteringNode::ArucoFilteringNode() 
: loop_rate{rate}
{

    arucoPoseSub = node.subscribe("/aruco_tf_poses_out", 10, &ArucoFilteringNode::aruco_cb, this);
    dvlAltSub = node.subscribe("/dvl/altitude", 10, &ArucoFilteringNode::dvl_alt_cb, this);
    
    opPosePubTf         = node.advertise<geometry_msgs::PoseStamped   >("aruco_filtered_poses_out"  ,100);
    opPosePubTfLandmark = node.advertise<vortex_msgs  ::ObjectPosition>("object_filtered_positions_in" ,100);


    // Time and samples
    const double system_dt = rate*0.001;
    const double measurement_dt = rate*0.005;

    int N = static_cast<int>(simulation_time / system_dt);
    int M = static_cast<int>(measurement_dt / system_dt);
}

// Process function as standard function
vec AprocessFunction(const vec& q, const vec& u) {
  vec q_pred = vec(2).zeros();

  q_pred(0) = q(0) + q(1) * system_dt;
  q_pred(1) = q(1) + (m * (u(0) - g) * d * sin(q(0)) - b * q(1)) * system_dt /
              (m * d * d);

  return q_pred;
}

void ArucoFilteringNode::aruco_cb(const geometry_msgs::PoseStamped& msg_in)
{
    m_aruco_position = Eigen::Vector3d{msg_in.pose.position.x, msg_in.pose.position.y, msg_in.pose.position.z};

}

void ArucoFilteringNode::dvl_alt_cb(const std_msgs::Float32& msg_in)
{
    float m_dvl_alt = msg_in.data;
}

void ArucoFilteringNode::publishPose(const geometry_msgs::Pose& pose, ros::Time timestamp)
{
    static size_t counter{0};
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "zed2i_left_camera_frame";
    poseMsg.header.seq = counter++;
    poseMsg.header.stamp = timestamp; // Should the time now be used, or the time the image was taken be used?
    poseMsg.pose = pose;
    opPosePub.publish(poseMsg);
    
    // Transform udfc pose to world frame


    try {
        odom_udfc_transform = tfBuffer.lookupTransform("odom", "udfc_link", timestamp);
    }
    catch(tf2::TransformException &ex) {
        ROS_WARN_STREAM(ex.what());
    }

    geometry_msgs::Pose poseTF;
    tf2::doTransform(pose, poseTF, odom_udfc_transform);


    geometry_msgs::PoseStamped poseTFMsg;
    poseTFMsg.header.frame_id = "odom";
    poseTFMsg.header.seq      = counter;
    poseTFMsg.header.stamp    = timestamp;
    poseTFMsg.pose            = poseTF;
    opPosePubTf.publish(poseTFMsg);

    vortex_msgs::ObjectPosition vortexPoseMsg;
    vortexPoseMsg.objectID          = "docking_point";
    vortexPoseMsg.objectPose        = poseTFMsg;
    vortexPoseMsg.isDetected        = true;
    vortexPoseMsg.estimateConverged = true;
    vortexPoseMsg.estimateFucked    = false;
    opPosePubTfLandmark.publish(vortexPoseMsg);
}

void ArucoFilteringNode::execute()
{
    while (ros::ok()) {

        // // IMAGE INPUT
        // cv::Mat img = cv::imread("/vortex_ws/src/vortex-cv/aruco_Filtering/test/pictures/TACboard3.jpg", cv::IMREAD_COLOR);
        // if( img.empty() )  // Check for invalid input
        // {
        //     ROS_INFO("Could not open or find the image");
        // }

        // // WEBCAM INPUT
        // static cv::Mat img;

        // // cv::namedWindow("Display window");
        // static cv::VideoCapture cap(0);
        // if (!cap.isOpened()) {
        //     ROS_INFO("cannot open camera");
        // }   
        // cap >> img;


        // geometry_msgs::Pose pose;
        // int markersDetected = arucoHandler.detectBoardPose(img, board, pose);
        // publishCVImg(img, ros::Time::now());
        // if (markersDetected > 0) 
        // {
        //     publishPose(pose, ros::Time::now());
        // }


        ros::spinOnce();
        loop_rate.sleep();
    }
}





int main(int argc, char **argv) {
    ros::init(argc,argv,"ArucoFilteringNode");

    ArucoHandler arucoHandler;

    ArucoFilteringNode arucoNode;
    arucoNode.execute();
}


// Process function as standard function
Eigen::Vector3d processFunction(const Eigen::& q, const vec& u) {
  vec q_pred = vec(2).zeros();

  q_pred(0) = q(0) + q(1) * system_dt;
  q_pred(1) = q(1) + (m * (u(0) - g) * d * sin(q(0)) - b * q(1)) * system_dt /
              (m * d * d);

  return q_pred;
}

// Output function as lambda
auto outputFunction = [](const vec& q)->vec{
  return {d * sin(q(0)), d * cos(q(0))}; };

// Process Jacobian as member function
struct ProcessJacobian {
  mat processJacobian(const vec& q, const vec& u) {
    double a11 = 1.0;
    double a12 = system_dt;
    double a21 = (u(0) - g) * cos(q(0)) * system_dt / d;
    double a22 = 1.0 - b * system_dt / (m * d * d);

    return { {a11, a12},
             {a21, a22} };
  }
};

// Output Jacobian as function object
struct outputJacobian {
  mat operator()(const vec& q) const {
    return { { d * cos(q(0)), 0.0},
             {-d * sin(q(0)), 0.0} };
  }
};