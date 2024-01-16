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

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
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
    #include "target_msgs/TargetJoint.h"
    #include "target_msgs/TargetCartesian.h"
    #include "target_msgs/TargetData.h"

    // Target Services
    // #include "target_msgs/CreateTarget.h"
    // #include "target_msgs/UpdateTarget.h"

    // Target Context
    #include "target_msgs/target_context.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Manager Class
    // -------------------------------
    /** \brief Robot system target handler
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


            // Load Targets Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom targets from the parameter server.
            *
            * Parameter information for each target is stored in the local collective parameter vector
            * (param_vec_) [std::vector<XmlRpc::XmlRpcValue>].
            *
            * \param param_name Targets parameter name, located on parameter server [std::string]
            * \return Function return: Successful/Unsuccessful (true/false) [bool]
            */
            bool loadParamData(
                const std::string& param_name);


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


            // Service Callback: Create Target object
            // -------------------------------
            /** \brief Create target object callback function.
            *
            * Service callback function for creating a custom Target object.
            * Requires information on target-name, target-type and target data
            * related to specified target-type. 
            * The following target-types are supported:
            * 1) Joint Target
            * 2) Cartesian Target
            *
            * \param request    Service-Request [target_msgs::CreateTarget::Request]
            * \param response   Service-Response [target_msgs::CreateTarget::Response]
            * \return Function result: Successful/Unsuccessful (true/false)
            */
            bool createTargetObjectCB(
                target_msgs::CreateTarget::Request& req,
                target_msgs::CreateTarget::Response& res);

            
            // Service Callback: Update Target object
            // -------------------------------
            /** \brief Update target object callback function.
            *
            * Service callback function for updating a custom target object.
            * Function requires information on name of the target-name to update.
            * The following parameters can be updated: reference frame and pose of the user-frame.
            *
            * \param request    Service-Request [target_msgs::UpdateTarget::Request]
            * \param response   Service-Response [target_msgs::UpdateTarget::Response]
            * \return Function result: Successful/Unsuccessful (true/false)
            */
            bool updateTargetObjectCB(
                target_msgs::CreateTarget::Request& req,
                target_msgs::CreateTarget::Response& res);
    }; // End Class: TargetManager
} // End Namespace: Target
#endif // TARGET_MANAGER_H 