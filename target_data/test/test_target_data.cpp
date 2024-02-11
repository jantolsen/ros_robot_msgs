// Test Target Data Node 
// -------------------------------
// Description:
//      Test Target Data Node
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
    #include "target_data/TargetData.h"
    #include "target_data/TargetJoint.h"
    #include "target_data/TargetCartesian.h"
    #include "target_data/TargetJointExtAxis.h"

    // Target
    #include "target_data/target_base.h"
    #include "target_data/target_manager.h"


// Target Test: 1
// -------------------------------
void targetTest1(ros::NodeHandle nh)
{
    // Define and initialize Target-Handler
    std::string param_name = "targets_test/test1";
    XmlRpc::XmlRpcValue param_xml;

    // TargetCartesian
    Target::TargetCartesian targetObject(nh, param_name);

    // Debug Print
    targetObject.printTargetData();
}


// Target Test: 2
// -------------------------------
void targetTest2(ros::NodeHandle nh)
{
    // Define and initialize Target-Handler
    std::string param_name = "/targets_test/test2";
    XmlRpc::XmlRpcValue param_xml;
    
    // Check parameter server for parameter-data
    if(!ros::param::get(param_name, param_xml))
    {
        // Failed to get parameter
        ROS_ERROR_STREAM(__FUNCTION__ << "Failed!");
        return;
    }

    // TargetCartesian
    Target::TargetCartesian targetObject(nh, param_xml);

    // Debug Print
    targetObject.printTargetData();
}


// Target Test: 3
// -------------------------------
void targetTest3(ros::NodeHandle nh)
{
    // Define and initialize Target-Handler
    std::string param_name = "targets_test/test3";
    XmlRpc::XmlRpcValue param_xml;

    // TargetJoint
    Target::TargetJoint targetObject(nh, param_name);

    // Debug Print
    targetObject.printTargetData();
}


// Target Test: 4
// -------------------------------
void targetTest4(ros::NodeHandle nh)
{
    // Define and initialize Target-Handler
    std::string param_name = "/targets_test/test4";
    XmlRpc::XmlRpcValue param_xml;
    
    // Check parameter server for parameter-data
    if(!ros::param::get(param_name, param_xml))
    {
        // Failed to get parameter
        ROS_ERROR_STREAM(__FUNCTION__ << "Failed!");
        return;
    }

    // TargetJoint
    Target::TargetJoint targetObject(nh, param_name);

    // Debug Print
    targetObject.printTargetData();
}


// Target Test: X
// -------------------------------
void targetTestX(ros::NodeHandle nh)
{
    // Define User-Frame Msgs
    target_data::TargetData targetDataMsg;

    // Assign parameters
    targetDataMsg.header.name = "testX";
    targetDataMsg.header.type_name = "JOINT";
    targetDataMsg.header.type = static_cast<int>(Target::TargetType::CARTESIAN);

    // Assign cartesian data of target
    targetDataMsg.cartesian.ref_frame = "world";
    targetDataMsg.cartesian.pose_config.position.x = -1.0;
    targetDataMsg.cartesian.pose_config.position.y = 5.8;
    targetDataMsg.cartesian.pose_config.position.z = 2.3;
    targetDataMsg.cartesian.pose_config.orientation.x = -4.3;
    targetDataMsg.cartesian.pose_config.orientation.y = 8.0;
    targetDataMsg.cartesian.pose_config.orientation.z = 3.14;
    targetDataMsg.cartesian.pose = Toolbox::Convert::poseRPYToPose(targetDataMsg.cartesian.pose_config);

    // Define and initialize Target-Handler
    Target::TargetBase target_X(nh, targetDataMsg);

    // Debug Print
    target_X.printTargetData();
}

// Target Manager
// -------------------------------
void targetManager(ros::NodeHandle nh, std::string param_name)
{   
    // Local Variables
    XmlRpc::XmlRpcValue param_xml;

    // Target Manager
    Target::TargetManager targetManager(nh, param_name);

    // Check parameter server for parameter-data
    if(!ros::param::get(param_name, param_xml))
    {
        // Failed to get parameter
        ROS_ERROR_STREAM(__FUNCTION__ << "Failed!");
        return;
    }

    // Target object
    auto targetObject = targetManager.createTargetObject(param_xml);

    // Debug Print
    ROS_WARN_STREAM( __FUNCTION__ 
        << ": Trying to print Target-Object from Target-manager");

    // Debug Print
    targetObject->printTargetData();
}

// Info-Kinemaatics Test Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_target_data");   

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
        targetTest1(nh);

    // Target #2
    // -------------------------------
        targetTest2(nh);


    // Target Manager
    // -------------------------------
        targetManager(nh, "/targets_test/test1");
        targetManager(nh, "/targets_test/test2");
        targetManager(nh, "/targets_test/test3");
    

    // Target #X
    // -------------------------------
        // targetTestX(nh);


    // Main Loop
    // -------------------------------
    ros::Rate rate(10.0);
    while (ros::ok()) 
    {
        // // Publish Target(s)
        // target_1.publishTarget();
        // target_2.publishTarget();
        // target_3.publishTarget();
        // target_4.publishTarget();
        // target_5.publishTarget();
        // target_6.publishTarget();
        // target_X.publishTarget();

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

