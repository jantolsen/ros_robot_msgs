// Target Context - Joint
// -------------------------------
// Description:
//      Robot system target-joint context
//      Collects information on custom target-joint from the parameter server. 
//      Structures and sorts the information into a target-joint message-type.
//      Provides functionality to interact and broadcast related target-joint information.
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
#ifndef TARGET_JOINT_CONTEXT_H       
#define TARGET_JOINT_CONTEXT_H

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
    #include "target_msgs/TargetData.h"
    #include "target_msgs/TargetExtAxis.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Enums
    // -------------------------------
    // Joint Axis Type 
    // (Matches the types defined in TargetExtAxis.msg)
    enum JointAxisType
    {
        ROTATION,
        LINEAR
    };


    // Target Joint Context Class
    // -------------------------------
    /** \brief Robot system target-joint context
    *
    * Collects information on custom target-joint from the parameter server. 
    * Structures and sorts the information into a target-joint message-type.
    * Provides functionality to interact and broadcast related target-joint information.
    */
    class TargetJointContext
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Define Shared-Pointer of Class-Object
            typedef typename std::shared_ptr<TargetJointContext> Ptr;

            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Joint Context class constuctor
            *
            * \param nh             ROS Nodehandle [ros::Nodehandle]
            * \param target_joint   Target-Joint data [target_msgs::TargetJoint]
            */
            TargetJointContext(
                ros::NodeHandle& nh,
                const target_msgs::TargetJoint& target_joint);


            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Joint Context class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_name Target-Joint parameter name, located on parameter server [std::string]
            */
            TargetJointContext(
                ros::NodeHandle& nh,
                const std::string& param_name);

            
            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Joint Context class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_xml  Target-Joint parameter, located on parameter server [XmlRpc::XmlRpcValue]
            */
            TargetJointContext(
                ros::NodeHandle& nh,
                const XmlRpc::XmlRpcValue& param_xml);


            // Class destructor
            // -------------------------------
            /** \brief Target-Joint Context class destructor
            */
            ~TargetJointContext();


            // Get Target-Joint Data
            // -------------------------------
            /** \brief Get information on custom target-joint data.
            *
            * Target-Joint data contains parameters and configuration for custom defined target-joint.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target-Joint data [target_msgs::TargetJoint]
            */
            target_msgs::TargetJoint getTargetJoint();


            // Update Target-Joint Data
            // -------------------------------
            /** \brief Update custom target-joint information.
            *
            * Target-Joint data contains parameters and configuration for custom defined target-joint.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \param target_joint Updated Target-Joint data [target_msgs::TargetJoint]
            */
            void updateTargetJoint(
                target_msgs::TargetJoint target_joint);


            // Load Target-Joint Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target-joint from the parameter server.
            *
            * Organize and structure the loaded parameters into target-joint message-type.
            * If successful, the gathered target-joint is returned. 
            * If parameter loading fails, function returns false.
            *
            * \param param_name Target-Joint parameter name, located on parameter server [std::string]
            * \return Function return: Successful: Target-Joint data [target_msgs::TargetJoint] / Unsuccessful: false [bool]
            */
            boost::optional<target_msgs::TargetJoint> loadParamData(
                const std::string& param_name);


            // Load Target-Joint Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target-joint from the parameter server.
            *
            * Organize and structure the loaded parameters into target message-type.
            * If successful, the gathered target-joint is returned. 
            * If parameter loading fails, function returns false.
            *
            * \param param_xml  Target-Joint parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Successful: Target-Joint data [target_msgs::TargetJoint] / Unsuccessful: false [bool]
            */
            boost::optional<target_msgs::TargetJoint> loadParamData(
                const XmlRpc::XmlRpcValue& param_xml);

            
            // Print Target-Joint Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Print information on target-joint data to terminal.
            *
            * Implemented for debugging purposes.
            */
            void printTargetJoint();


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Print Target-Joint Data 
            // (with External Axis)
            // -------------------------------
            // (Function Overloading)
            /** \brief Print information on target-joint data (with external-axis) to terminal.
            *
            * Implemented for debugging purposes.
            */
            void printTargetJointExtAxis();


            // Get Target-Joint Parameter Data
            // -------------------------------
            /** \brief Reads and loads information on target-joint from the parameter server.
            *
            * Organize and structure the loaded parameters into target-joint message-type.
            * If successful, the gathered target-joint data is returned.
            * If parameter loading fails, an error message is given and a runtime expection is thrown.
            *
            * \param param_xml  Target-Joint parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Successful: target-joint data [target_msgs::TargetJoint] / Unsuccessful: false [bool]
            */
            target_msgs::TargetJoint getParamTargetJoint(
                const XmlRpc::XmlRpcValue& param_xml);

            
            // Get Target-Joint Parameter Data 
            // (with External Axis)
            // -------------------------------
            /** \brief Reads and loads information on target-joint (with external-axis) from the parameter server.
            *
            * Organize and structure the loaded parameters into target-joint message-type.
            * If successful, the gathered target-joint data is returned.
            * If parameter loading fails, an error message is given and a runtime expection is thrown.
            *
            * \param param_xml      Target-Joint parameters [XmlRpc::XmlRpcValue]
            * \param ext_axis_data  External-Axis data [target_msgs::TargetExtAxis]
            * \return Function return: Successful: target-joint data [target_msgs::TargetJoint] / Unsuccessful: false [bool]
            */
            target_msgs::TargetJoint getParamTargetJointExtAxis(
                const XmlRpc::XmlRpcValue& param_xml,
                const target_msgs::TargetExtAxis& ext_axis_data);


            // Get External Axis Parameter Data
            // -------------------------------
            /** \brief Reads and loads information on External-Axis from the parameter server.
            *
            * Organize and structure the loaded parameters into external-axis message-type.
            * If successful, the gathered external-axis data is returned. 
            * If parameter loading fails, an error message is given and a runtime expection is thrown..
            *
            * \param param_xml  Target parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Successful: target data [target_msgs::TargetExtAxis] / Unsuccessful: false [bool]
            */
            target_msgs::TargetExtAxis getParamExtAxis(
                const XmlRpc::XmlRpcValue& param_xml);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Class Local Member(s)
            // -------------------------------
            target_msgs::TargetJoint target_joint_;

            // ROS Nodehandle(s)
            // -------------------------------
            ros::NodeHandle nh_;


            // Initialize Joint Axis Type Map
            // -------------------------------
            /** \brief Initialize joint axis type map
            *
            * Each joint-axis-type [JointAxisType] is paired with a name [std::string],
            * where the entries of joint-axis-types matches the defined types in TargetExtAxis.msg
            * Function populates and returns the joint-axis-type map [std::map<std::string, JointAxisType>]
            *
            * \return Target-Type Map [std::map<std::string, JointAxisType>]
            */
            static std::map<std::string, JointAxisType> initJointAxisTypeMap();


            // Initialize Joint Axis Type Names
            // -------------------------------
            /** \brief Initialize External Axis type names
            *
            * Each joint-axis-type [JointAxisType] is paired with a name [std::string] in a map.
            * Function iterates over the given joint-axis-type map and stores the joint-axis-type names in a vector.
            *
            * \param joint_type_map Joint-Axis-type Map [std::map<std::string, JointAxisType>]
            * \return Joint Axis-Type Names [std::vector<std::string>]
            */
            static std::vector<std::string> initJointAxisTypeNames(
                std::map<std::string, JointAxisType> joint_type_map);


    }; // End Class: TargetJointContext
} // End Namespace: Target
#endif // TARGET_JOINT_CONTEXT_H 