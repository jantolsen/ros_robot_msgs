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

// Include guard:
// -------------------------------
#ifndef TARGET_MANAGER_H       
#define TARGET_MANAGER_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>
    #include <map>
    #include <memory>

    // Ros
    #include <ros/ros.h>

    // TF2
    #include <tf2_ros/transform_broadcaster.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Target-Data
    #include "target_data/target_base.h"
    // #include "target_data/target_joint.h"
    // #include "target_data/target_cartesian.h"
    // #include "target_data/target_joint_extaxis.h"

    // Target Messages
    #include "target_data/TargetHeader.h"
    #include "target_data/TargetData.h"
    #include "target_data/TargetJoint.h"
    #include "target_data/TargetCartesian.h"
    #include "target_data/TargetJointExtAxis.h"


// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Manager Class
    // -------------------------------
    /** \brief Robot system target manager
    *
    * Collects, sorts and structures information on custom defined targets,
    * where parameter and configuration data is gathered from parameter server. 
    * The manager utilizes the target-base class to create target-objects based
    * on the configured target-type. The manager provides functionality for accessing 
    * and publishing information for the target-objects.
    * 
    * The target manager is implemented as a part of the strategy-pattern implementation 
    * of the robot-target. The target-base defines the common behaviour and methods
    * for all strategies (target-types). The related strategies must then implement this interface
    * to provide their unique implementation of the behaviour and methods. 
    * The manager is reponsible for creating of the targets using the related strategy.
    */
    class TargetManager
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Class constructor
            // -------------------------------
            /** \brief Target Manager class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_name Parameter name for collective targets, located on parameter server [std::string]
            */
            TargetManager(
                ros::NodeHandle& nh,
                const std::string& param_name);


            // Class destructor
            // -------------------------------
            /** \brief Target Manager class destructor
            */
            ~TargetManager();


            // Get This
            // -------------------------------
            /** \brief Get Target-Manager object pointer
            *
            * \return Pointer to Target-Base object [TargetBase*]
            */
            TargetManager* getThis();


            // Create Target Object
            // -------------------------------
            // (Function Overloading)
            /** \brief Create a Target object
            *
            * Creates a target object [TargetBase] from given parameter data.
            * Returns the created object as a shared-pointer
            *
            * \param target_data    Target data (collection of parameter data) [target_data::TargetData]
            * \return Shared-Pointer of a Target object [std::shared_ptr<TargetBase>]
            */
            std::shared_ptr<TargetBase> createTargetObject(
                const target_data::TargetData target_data);


            // Create Target Object
            // -------------------------------
            // (Function Overloading)
            /** \brief Create a User-Frame object
            *
            * Creates a target object [TargetBase] from given parameter data.
            * Returns the created object as a shared-pointer
            *
            * \param target_param_xml   Target parameter data [XmlRpc::XmlRpcValue]
            * \return Shared-Pointer of a Target object [std::shared_ptr<TargetBase>]
            */
            std::shared_ptr<TargetBase> createTargetObject(
                const XmlRpc::XmlRpcValue target_param_xml);


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Initialize Target Manager
            // -------------------------------
            /** \brief Initialize Target Manager
            */
            void init();


            // Determine Target-Type
            // -------------------------------
            /** \brief Determine Target-Type of given target parameter
            */
            TargetType determineTargetType(
                const XmlRpc::XmlRpcValue target_param_xml);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Local class member(s)
            // -------------------------------
            XmlRpc::XmlRpcValue param_name_;
            std::vector<XmlRpc::XmlRpcValue> param_vec_;
            std::shared_ptr<TargetBase> targetObject_;
            std::map<std::string, std::shared_ptr<TargetBase>> target_map_;
            std::vector<std::shared_ptr<TargetBase>> target_vec_;

            // ROS Nodehandle(s)
            // -------------------------------
            ros::NodeHandle nh_;

            // // ROS Service Server(s)
            // // -------------------------------
            // // (Declaring Service-Servers as shared-pointers to allow shared ownership of the objects)
            // std::shared_ptr<ros::ServiceServer> createTargetObject_server_;  // ROS Server for creating a Target object
            // std::shared_ptr<ros::ServiceServer> updateTargetObject_server_;  // ROS Server for updating Target object

            // // Service Callback: Create Target object
            // // -------------------------------
            // /** \brief Create target object callback function.
            // *
            // * Service callback function for creating a custom Target object.
            // * Requires information on target-name, target-type and target data
            // * related to specified target-type. 
            // * The following target-types are supported:
            // * 1) Joint Target
            // * 2) Cartesian Target
            // *
            // * \param request    Service-Request [target_data::CreateTarget::Request]
            // * \param response   Service-Response [target_data::CreateTarget::Response]
            // * \return Function result: Successful/Unsuccessful (true/false)
            // */
            // bool createTargetObjectCB(
            //     target_data::CreateTarget::Request& req,
            //     target_data::CreateTarget::Response& res);

            
            // // Service Callback: Update Target object
            // // -------------------------------
            // /** \brief Update target object callback function.
            // *
            // * Service callback function for updating a custom target object.
            // * Function requires information on name of the target-name to update.
            // * The following parameters can be updated: reference frame and pose of the user-frame.
            // *
            // * \param request    Service-Request [target_data::UpdateTarget::Request]
            // * \param response   Service-Response [target_data::UpdateTarget::Response]
            // * \return Function result: Successful/Unsuccessful (true/false)
            // */
            // bool updateTargetObjectCB(
            //     target_data::CreateTarget::Request& req,
            //     target_data::CreateTarget::Response& res);
    }; // End Class: TargetManager
} // End Namespace: Target
#endif // TARGET_MANAGER_H 