#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv) {
    std::string parent_frame = "odom"; 
    std::string child_frame = "udfc_link";

    // Create a point transformer object
    geometry_msgs::TransformStamped odom_udfc_transform;

    // Create a buffer, listener, and broadcaster for transformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    tf2_ros::TransformBroadcaster tfBroadcaster;

    // Initialize the node, wait for a transform to be available
    while (!tfBuffer.canTransform(parent_frame, child_frame, ros::Time(0))) {
        try {
            ROS_INFO_STREAM("No transform between " << parent_frame << " and " << child_frame);
            ros::Duration(2.0).sleep();
        }
        catch(tf2::TransformException &ex) {
            ROS_WARN_STREAM("TransformException: " << ex.what());
            ros::Duration(2.0).sleep();
        }
    }
    ROS_INFO_STREAM("Transform between " << parent_frame << " and " << child_frame << " found.");

    geometry_msgs::Pose inputPose;
    geometry_msgs::Pose outputPose;

    tf2::doTransform(inputPose, outputPose, odom_udfc_transform);
};



// Python implementation of function to use the transform

// def map_to_odom(self, point_cam, trans_udfc_odom, dp_ref=None):

//         """
//         Takes in a point in homogenious camera coordinates and calculates the X, Y, Z in odom frame using the similarity 
//         triangles equations through the dp assumptions of no roll and pitch and known height over ground.

//         Input: 
//             point_cam - [x_tilde, y_tilde, 1]^T
//             trans_udfc_odom - position of udfc in odom

//         Output:
//             point_odom - estimate of point in odom frame [X, Y, Z]^T
//         """

//         z_over_path = self.H_pool_prior - abs(trans_udfc_odom[2]) - self.h_path_prior
        
//         # Map X and Y to world frame
//         X = z_over_path * point_cam[0]
//         Y = z_over_path * point_cam[1]

//         if dp_ref:
//             point_odom = np.array([X, Y, self.Z_prior_ref]) + trans_udfc_odom
//         else:
//             point_odom = np.array([X, Y, z_over_path]) + trans_udfc_odom
//         return point_odom
