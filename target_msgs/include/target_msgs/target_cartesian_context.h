// Target Context - Cartesian
// -------------------------------
// Description:
//      Robot system target-cartesian context
//      Collects information on custom target-cartesian from the parameter server. 
//      Structures and sorts the information into a target-cartesian message-type.
//      Provides functionality to interact and broadcast related target-cartesian information.
//
// Version:
//  0.2 -   Overhaul of Target-Context implementation.
//          Split target-type into seperate classes.
//          [16.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [09.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
#ifndef TARGET_CARTESIAN_CONTEXT_H       
#define TARGET_CARTESIAN_CONTEXT_H

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
    #include "target_msgs/TargetCartesian.h"
    #include "target_msgs/TargetData.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Joint Context Class
    // -------------------------------
    /** \brief Robot system target-cartesian context
    *
    * Collects information on custom target-cartesian from the parameter server. 
    * Structures and sorts the information into a target-cartesian message-type.
    * Provides functionality to interact and broadcast related target-cartesian information.
    */
    class TargetCartesianContext
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Define Shared-Pointer of Class-Object
            typedef typename std::shared_ptr<TargetCartesianContext> Ptr;

            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Cartesian Context class constuctor
            *
            * \param nh                 ROS Nodehandle [ros::Nodehandle]
            * \param target_cartesian   Target-Cartesian data [target_msgs::TargetCartesian]
            */
            TargetCartesianContext(
                ros::NodeHandle& nh,
                const target_msgs::TargetCartesian& target_cartesian);


            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Cartesian Context class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_name Target-Cartesian parameter name, located on parameter server [std::string]
            */
            TargetCartesianContext(
                ros::NodeHandle& nh,
                const std::string& param_name);

            
            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Cartesian Context class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_xml  Target-Cartesian parameter, located on parameter server [XmlRpc::XmlRpcValue]
            */
            TargetCartesianContext(
                ros::NodeHandle& nh,
                const XmlRpc::XmlRpcValue& param_xml);


            // Class destructor
            // -------------------------------
            /** \brief Target-Cartesian Context class destructor
            */
            ~TargetCartesianContext();


            // Get Target-Cartesian Data
            // -------------------------------
            /** \brief Get information on custom target-cartesian data.
            *
            * Target-Cartesian data contains parameters and configuration for custom defined target-cartesian.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target-Cartesian data [target_msgs::TargetCartesian]
            */
            target_msgs::TargetCartesian getTargetCartesianData();


            // Update Target-Cartesian Data
            // -------------------------------
            /** \brief Update custom target-cartesian information.
            *
            * Target-Cartesian data contains parameters and configuration for custom defined target-cartesian.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \param target_cartesian   Updated Target-Cartesian data [target_msgs::TargetCartesian]
            */
            void updateTargetData(
                target_msgs::TargetCartesian target_cartesian);


            // Print Target-Cartesian Data
            // -------------------------------
            /** \brief Print information on target-cartesian data to terminal.
            *
            * Implemented for debugging purposes.
            */
            void printTargetCartesian();


            // Load Target-Cartesian Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target-cartesian from the parameter server.
            *
            * Organize and structure the loaded parameters into target-cartesian message-type.
            * If successful, the gathered target-cartesian is returned. 
            * If parameter loading fails, function returns false.
            *
            * \param param_name Target-Cartesian parameter name, located on parameter server [std::string]
            * \return Function return: Successful: Target-Cartesian data [target_msgs::TargetCartesian] / Unsuccessful: false [bool]
            */
            boost::optional<target_msgs::TargetCartesian> loadParamData(
                const std::string& param_name);


            // Load Target-Cartesian Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target-cartesian from the parameter server.
            *
            * Organize and structure the loaded parameters into target message-type.
            * If successful, the gathered target-cartesian is returned. 
            * If parameter loading fails, function returns false.
            *
            * \param param_xml  Target-Cartesian parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Successful: Target-Cartesian data [target_msgs::TargetCartesian] / Unsuccessful: false [bool]
            */
            boost::optional<target_msgs::TargetCartesian> loadParamData(
                const XmlRpc::XmlRpcValue& param_xml);

            
        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Get Target-Cartesian Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on target-cartesian from the parameter server.
            *
            * Organize and structure the loaded parameters into target-cartesian message-type.
            * If successful, the gathered target-cartesian data is returned.
            * If parameter loading fails, an error message is given and a runtime expection is thrown.
            *
            * \param param_xml  Target-Cartesian parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Successful: target-cartesian data [target_msgs::TargetCartesian] / Unsuccessful: false [bool]
            */
            target_msgs::TargetCartesian getParamTargetCartesian(
                const XmlRpc::XmlRpcValue& param_xml);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Class Local Member(s)
            // -------------------------------
            target_msgs::TargetCartesian target_cartesian;

            // ROS Nodehandle(s)
            // -------------------------------
            ros::NodeHandle nh_;

    }; // End Class: TargetCartesianContext
} // End Namespace: Target
#endif // TARGET_CARTESIAN_CONTEXT_H 