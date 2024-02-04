// Target Manager
// -------------------------------
// Description:
//      Robot system target manager.
//      Collects, sorts and structures information on custom defined targets,
//      where parameter and configuration data is gathered from parameter server. 
//      The manager utilizes the target-base class to create target-objects based
//      on the configured target-type. The manager provides functionality for accessing 
//      and publishing information for the target-objects.
// 
//      The target manager is implemented as a part of the strategy-pattern implementation 
//      of the robot-target. The target-base defines the common behaviour and methods
//      for all strategies (target-types). The related strategies must then implement this interface
//      to provide their unique implementation of the behaviour and methods. 
//      The manager is reponsible for creating of the targets using the related strategy.
//
// Version:
//  0.2 -   Overhaul of Target implementation.
//          Introduding strategy-pattern.
//          Utilizing interface and target-types (strategies) into seperate classes.
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

            // Initialize Target-Manager
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

            
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing finished");
        } // Function End: init()


        // // Create Target Object
        // // -------------------------------
        // // (Function Overloading)
        // std::shared_ptr<TargetBase> createTargetObject(
        //     const target_data::TargetData target_data)
        // {
            
        // } // Function End: createTargetObject()


        // Create Target Object
        // -------------------------------
        // (Function Overloading)
        std::shared_ptr<TargetBase> TargetManager::createTargetObject(
            const XmlRpc::XmlRpcValue target_param_xml)
        {
            target_data::TargetData targetData;

            auto result_target = targetObject_->loadParamData(target_param_xml);;
            if (!result_target)
            {
                targetData = result_target.value();
            }

            // Determine Target-Type
            

            // Determine target-type an create appropriate strategy
            switch (targetData.header.type)
            {
                case TargetType::JOINT:
                    targetObject_ = std::make_shared<TargetJoint>(nh_, targetData);
                    break;

                case TargetType::CARTESIAN:
                    targetObject_ = std::make_shared<TargetCartesian>(nh_, targetData);
                    break;

                case TargetType::JOINT_EXTAXIS:
                    targetObject_ = std::make_shared<TargetJointExtAxis>(nh_, targetData);
                    break;

                default:
                    break;
            }
            
            // Function return
            return targetObject_;
        } // Function End: createTargetObject()

} // End Namespace: Target