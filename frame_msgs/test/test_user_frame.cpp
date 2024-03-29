// Test User-Frame Node 
// -------------------------------
// Description:
//      Test User-Frame Node
//
// Version:
//  0.1 - Initial Version
//        [04.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Frame Messages
    #include "frame_msgs/UserFrame.h"

    // User-Frame
    #include "frame_msgs/user_frame_manager.h"
    #include "frame_msgs/user_frame_context.h"


// Info-Kinemaatics Test Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_userframe_handler");   

    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Test(s)
    // -------------------------------
        std::string param_name;
        XmlRpc::XmlRpcValue param_xml;


    // User-Frame Manager
    // -------------------------------
        // Define and initialize User-Frame Manager
        Frame::UserFrameManager userFrameManager(nh, "/user_frames");


    // User-Frame #1
    // -------------------------------
        // Define and initialize User-Frame-Handler
        param_name = "/user_frames_test/test1";
        Frame::UserFrameContext uf_1(nh, param_name);

        // Debug Print
        uf_1.printUserFrameData();

    // User-Frame #2
    // -------------------------------
        // Define and initialize User-Frame-Handler
        param_name = "/user_frames_test/test2";
        
        // Check parameter server for Information-Kinematics parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM("Failed! User-Frames Parameter [" << param_name << "] not found");

            // Function return
            return false;
        }

        Frame::UserFrameContext uf_2(nh, param_xml);

        // Debug Print
        uf_2.printUserFrameData();

    // User-Frame #3
    // -------------------------------
        // Define User-Frame Msgs
        frame_msgs::UserFrame userFrameMsg;

        // Assign parameters
        userFrameMsg.name = "test3";
        userFrameMsg.ref_frame = "world";
        userFrameMsg.pose_rpy.position.x = -1.0;
        userFrameMsg.pose_rpy.position.y = 5.8;
        userFrameMsg.pose_rpy.position.z = 2.3;
        userFrameMsg.pose_rpy.orientation.x = -4.3;
        userFrameMsg.pose_rpy.orientation.y = 8.0;
        userFrameMsg.pose_rpy.orientation.z = 3.14;

        // Assign Transform data of User-Frame
        geometry_msgs::Pose pose = Toolbox::Convert::poseRPYToPose(userFrameMsg.pose_rpy);
        userFrameMsg.transform_stamped = Toolbox::Convert::poseToTransform(pose, userFrameMsg.ref_frame, userFrameMsg.name);

        // Define and initialize User-Frame-Handler
        Frame::UserFrameContext uf_3(nh, userFrameMsg);

        // Debug Print
        uf_3.printUserFrameData();


    // Main Loop
    // -------------------------------
    ros::Rate rate(10.0);
    while (ros::ok()) 
    {
        // // Publish User-Frame
        // uf_1.publishUserFrame();
        // uf_2.publishUserFrame();
        // uf_3.publishUserFrame();

        // // Broadcast User-Frame
        // uf_1.broadcastUserFrame();
        // uf_2.broadcastUserFrame();
        // uf_3.broadcastUserFrame();

        userFrameManager.publishAndBroadcastUserFrames();

        ros::spinOnce(); // Handle ROS callbacks
    }
    
    // // Spin to keep the node alive
    // ros::spin();
    
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}

