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

    // Target
    #include "target_msgs/target_context.h"
    #include "target_msgs/target_manager.h"

// Test: Function
// -------------------------------
void testFunc()
{
    
    
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
        std::string param_name;
        XmlRpc::XmlRpcValue param_xml;
    
    // Target #1
    // -------------------------------
        // Define and initialize Target-Handler
        param_name = "/targets_test/test1";
        Target::TargetContext target_1(nh, param_name);

        // Debug Print
        target_1.printTargetData();

    // Target #2
    // -------------------------------
        // Define and initialize Target-Handler
        param_name = "/targets_test/test2";
        
        // Check parameter server for Information-Kinematics parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM("Failed! User-Frames Parameter [" << param_name << "] not found");

            // Function return
            return false;
        }

        Target::TargetContext target_2(nh, param_xml);

    // Target #3
    // -------------------------------
        // Define and initialize Target-Handler
        param_name = "/targets_test/test3";
        Target::TargetContext target_3(nh, param_name);

        // Debug Print
        target_3.printTargetData();

    // Target #4
    // -------------------------------
        // Define and initialize Target-Handler
        param_name = "/targets_test/test4";
        Target::TargetContext target_4(nh, param_name);

        // Debug Print
        target_4.printTargetData();

    // Target #5
    // -------------------------------
        // Define and initialize Target-Handler
        param_name = "/targets_test/test5";
        Target::TargetContext target_5(nh, param_name);

        // Debug Print
        target_5.printTargetData();

    // Target #6
    // -------------------------------
        // Define and initialize Target-Handler
        param_name = "/targets_test/test6";
        Target::TargetContext target_6(nh, param_name);

        // Debug Print
        target_6.printTargetData();
    

    // Target #X
    // -------------------------------
        // Define User-Frame Msgs
        target_msgs::TargetData targetDataMsg;

        // Assign parameters
        targetDataMsg.name = "testX";
        targetDataMsg.type_name = "JOINT";
        targetDataMsg.type = static_cast<int>(Target::TargetType::CARTESIAN);
        targetDataMsg.visible = true;

        // Assign cartesian data of target
        targetDataMsg.cartesian.ref_frame = "world";
        targetDataMsg.cartesian.pose_rpy.position.x = -1.0;
        targetDataMsg.cartesian.pose_rpy.position.y = 5.8;
        targetDataMsg.cartesian.pose_rpy.position.z = 2.3;
        targetDataMsg.cartesian.pose_rpy.orientation.x = -4.3;
        targetDataMsg.cartesian.pose_rpy.orientation.y = 8.0;
        targetDataMsg.cartesian.pose_rpy.orientation.z = 3.14;
        targetDataMsg.cartesian.pose = Toolbox::Convert::poseRPYToPose(targetDataMsg.cartesian.pose_rpy);

        // Define and initialize Target-Handler
        Target::TargetContext target_X(nh, targetDataMsg);

        // Debug Print
        target_X.printTargetData();


    // Main Loop
    // -------------------------------
    ros::Rate rate(10.0);
    while (ros::ok()) 
    {
        // Publish Target(s)
        target_1.publishTarget();
        target_2.publishTarget();
        target_3.publishTarget();
        target_4.publishTarget();
        target_5.publishTarget();
        target_6.publishTarget();
        target_X.publishTarget();

        // userFrameManager.publishAndBroadcastUserFrames();

        ros::spinOnce(); // Handle ROS callbacks
    }

    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    // ros::waitForShutdown();

    // Function return
    return 0;
}

