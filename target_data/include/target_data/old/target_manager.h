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

    // Ros
    #include <ros/ros.h>

    // TF2
    #include <tf2_ros/transform_broadcaster.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Target Messages
    #include "target_data/TargetData.h"
    #include "target_data/TargetJoint.h"
    #include "target_data/TargetCartesian.h"
    #include "target_data/TargetJoint.h"

    // Target Services
    // #include "target_data/CreateTarget.h"
    // #include "target_data/UpdateTarget.h"

    // Target Context
    #include "target_data/target_context.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Manager Class
    // -------------------------------
    /** \brief Robot system target handler
    *
    * Collects, sorts and structures information on custom defined targets 
    * obtained from the parameter server. The manager utilizes the 
    * Target-Context class to store information as a target-message type 
    * for each defined target. Provides functionality for accessing, and publishing 
    * information on each target present in the system.
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
            std::map<std::string, std::shared_ptr<TargetContext>> target_map_;
            std::vector<std::shared_ptr<TargetContext>> target_vec_;


            // ROS Nodehandle(s)
            // -------------------------------
            ros::NodeHandle nh_;

            // ROS Service Server(s)
            // -------------------------------
            // (Declaring Service-Servers as shared-pointers to allow shared ownership of the objects)
            std::shared_ptr<ros::ServiceServer> createTargetObject_server_;  // ROS Server for creating a Target object
            std::shared_ptr<ros::ServiceServer> updateTargetObject_server_;  // ROS Server for updating Target object

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
