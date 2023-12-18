// Test Target Node 
// -------------------------------
// Description:
//      Test Target Node
//
// Version:
//  0.1 - Initial Version
//        [18.12.2023]  -   Jan T. Olsen
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

    // Target-Messages
    #include "target_msgs/TargetData.h"
    #include "target_msgs/TargetJoint.h"
    #include "target_msgs/TargetCartesian.h"

// Test: Function
// -------------------------------
void testFunc()
{
    target_msgs::TargetData target_data;
    target_msgs::TargetJoint target_joint;
    target_msgs::TargetCartesian target_cartesian;
    
} // Function end: testFunc()


// Info-Kinemaatics Test Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_target");   

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

