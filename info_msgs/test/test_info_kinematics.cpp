// Test Info-Kinematics Node 
// -------------------------------
// Description:
//      Test Info-Kinematics Node
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
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

    // Info-Kinematics
    #include "info_msgs/info_kinematics_handler.h"

    // Info-Messages
    #include "info_msgs/InfoKinematics.h"

// Test: Function
// -------------------------------
void testFunc()
{
    // Define Info-Kinematics Msgs
    info_msgs::InfoKinematics infoKinMsg;

    // Define and initialize Info-Kinematics-Handler
    Info::InfoKinematicsHandler kinHandler;

    // Load-Param
    kinHandler.test("/general/kinematics",infoKinMsg);

    // Debug Print
    // -------------------------------
    ROS_INFO(" ");
    ROS_INFO("Info-Kinematics:");
    ROS_INFO("--------------------");
    ROS_INFO_STREAM("Solver-Type: " << infoKinMsg.solver_type);
    ROS_INFO_STREAM("Solver-Name: " << infoKinMsg.solver_name);
    ROS_INFO_STREAM("Search-Resolution: " << infoKinMsg.search_resolution);
    ROS_INFO_STREAM("Timeout: " << infoKinMsg.timeout);
    ROS_INFO_STREAM("Attempts: " << infoKinMsg.attempts);
    ROS_INFO(" ");

} // Function end: testFunc()


// Info-Kinemaatics Test Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_info_kinematics");   

    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Test(s)
    // -------------------------------
    testFunc();
        
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    // ros::waitForShutdown();

    // Function return
    return 0;
}

