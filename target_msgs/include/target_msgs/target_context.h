// Target Context
// -------------------------------
// Description:
//      Robot system target context
//      Collects information on custom target from the parameter server. 
//      Structures and sorts the information into a target message-type.
//      Provides functionality to interact and broadcast related target information.
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
#ifndef TARGET_CONTEXT_H       
#define TARGET_CONTEXT_H

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


// Target Context Class
// -------------------------------
/** \brief Robot system target context
* Collects information on custom target from the parameter server. 
* Structures and sorts the information into a target message-type.
* Provides functionality to interact and broadcast related target information.
*/
class TargetContext
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:
        // Define Shared-Pointer of Class-Object
        typedef typename std::shared_ptr<TargetContext> Ptr;

            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target Context class constuctor
            *
            * \param nh             ROS Nodehandle [ros::Nodehandle]
            * \param target_data    Target data [target_msgs::TargetData]
            */
            TargetContext(
                ros::NodeHandle& nh,
                const target_msgs::TargetData& target_data);


            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target Context class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_name Target parameter name, located on parameter server [std::string]
            */
            TargetContext(
                ros::NodeHandle& nh,
                const std::string& param_name);

            
            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target Context class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_xml  Target parameter, located on parameter server [XmlRpc::XmlRpcValue]
            */
            TargetContext(
                ros::NodeHandle& nh,
                const XmlRpc::XmlRpcValue& param_xml);


            // Class destructor
            // -------------------------------
            /** \brief Target Context class destructor
            */
            ~TargetContext();


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Load Target Parameter Data
        // -------------------------------
        // (Function Overloading)
        /** \brief Reads and loads information on custom target from the parameter server.
        *
        * Organize and structure the loaded parameters into target message-type.
        * If successful, the gathered target is returned. 
        * Function returns fals if it fails to load target data.
        *
        * \param param_name Target parameter name, located on parameter server [std::string]
        * \return Function return: Successful: Target data [target_msgs::TargetData] / Unsuccessful: false [bool]
        */
        boost::optional<target_msgs::TargetData> loadParamData(
            const std::string& param_name);


        // Load target Parameter Data
        // -------------------------------
        // (Function Overloading)
        /** \brief Reads and loads information on custom target from the parameter server.
        *
        * Organize and structure the loaded parameters into target message-type.
        * If successful, the gathered target is returned. 
        * Function returns fals if it fails to load target data.
        *
        * \param param_xml  Target parameters [XmlRpc::XmlRpcValue]
        * \return Function return: Successful: target data [target_msgs::TargetData] / Unsuccessful: false [bool]
        */
        boost::optional<target_msgs::TargetData> loadParamData(
            const XmlRpc::XmlRpcValue& param_xml);

        
    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
        // Class-Name-Prefix for terminal message
        static const std::string CLASS_PREFIX;

        // Class Local Member(s)
        // -------------------------------
        target_msgs::TargetJoint target_joint_data_;
        target_msgs::TargetCartesian target_cartesian_data_;
        target_msgs::TargetData target_data_;

        // ROS Nodehandle(s)
        // -------------------------------
        ros::NodeHandle nh_;
        
        // ROS Publsiher(s)
        // -------------------------------
        ros::Publisher target_pub_;

}; // End Class: TargetContext
#endif // TARGET_CONTEXT_H 