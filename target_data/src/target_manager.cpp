// Target Manager
// -------------------------------
// Description:
//      Robot system target manager
//      Collects, sorts and structures information on custom defined targets 
//      obtained from the parameter server. The manager utilizes the 
//      Target-Context class to store information as a target-message type 
//      for each defined target. Provides functionality for accessing, and publishing 
//      information on each target present in the system.
//
// Version:
//  0.2 -   Overhaul of Target implementation.
//          Introduding strategy-pattern, utilizing interface, strategies and context
//          [20.01.2024]  -   Jan T. Olsen
//  0.2 -   Overhaul of Target implementation.
//          Split target-type into seperate classes.
//          [16.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [09.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "target_data/target_manager.h"

// Namespace: Target
// -------------------------------
namespace Target
{
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


        // Initialize Target Manager
        // -------------------------------
        void TargetManager::init()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing ... ");

            // // Target Parameter(s)
            // // -------------------------------
            //     // Initialize target parameter vector
            //     param_vec_.clear();

            //     // Load user-frames parameter data
            //     if(!loadParamData(param_name_))
            //     {
            //         // Report to terminal
            //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
            //             << ": Initialization failed!");

            //         // Function return
            //         return;
            //     }

            // // Target Object(s)
            // // -------------------------------
            //     // Iterate over target parameter vector and create target object for each entry
            //     for(std::size_t i = 0; i < param_vec_.size(); i++)
            //     {
            //         // Create user-frame object
            //         auto targetObject = createUserFrameObject(param_vec_[i]);

            //         // Debug
            //         userFrameObject->printUserFrameData();
            //     }

            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing finished");
        } // Function End: init()

} // End Namespace: Target