// Target Manager
// -------------------------------
// Description:
//      Robot system target handler
//      Collects, sorts and structures information on custom defined targets 
//      obtained from the parameter server. The manager utilizes the 
//      Target-Context class to store information as a target-message type 
//      for each defined target. Provides functionality for accessing, and publishing 
//      information on each target present in the system.
//
// Version:
//  0.1 - Initial Version
//        [09.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "target_msgs/target_manager.h"

// Target Manager Class
// -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetManager::CLASS_PREFIX = "TargetManager::";

    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetManager::TargetManager(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        nh_(nh),
        param_name_(param_name)
    {
        // Initialize Service-Server(s)
        // auto tmp_createTargetObject_server_ = nh_.advertiseService("/target/create", &TargetManager::createTargetObjectCB, this);
        // auto tmp_updateTargetObject_server_ = nh_.advertiseService("/target/update", &TargetManager::updateTargetObjectCB, this);
        // createTargetObject_server_ = std::make_shared<ros::ServiceServer>(tmp_createTargetObject_server_);
        // updateUserFrameObject_server_ = std::make_shared<ros::ServiceServer>(tmp_updateTargetObject_server_);

        // Initialize user-frame manager
        init();
    } // Class Constructor End: TargetManager()


    // Class Desctructor
    // -------------------------------
    TargetManager::~TargetManager()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~TargetManager()