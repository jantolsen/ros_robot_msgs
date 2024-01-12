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

// Namespace: Target
// -------------------------------
namespace Target
{
    // Enums
    // -------------------------------
    // Target Type 
    // (Matches the types defined in TargetData.msg)
    enum TargetType
    {
        JOINT,
        CARTESIAN
    };

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


            // Publish Target Data
            // -------------------------------
            /** \brief Publish target data. 
            *
            * Publishes target data as a topic.
            * The target is published as a [target_msgs::TargetData] message-type.
            */
            void publishTarget();


            // Get Target Data
            // -------------------------------
            /** \brief Get information on custom target data.
            *
            * Target data contains parameters and configuration for custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target data [target_msgs::TargetData]
            */
            target_msgs::TargetData getTargetData();


            // Update Target Data
            // -------------------------------
            /** \brief Update custom target information.
            *
            * Target data contains parameters and configuration for custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \param target_data Updated Target data [target_msgs::TargetData]
            */
            void updateTargetData(
                target_msgs::TargetData target_data);


            // Print Target Data
            // -------------------------------
            /** \brief Print information on target data to terminal.
            *
            * Implemented for debugging purposes.
            */
            void printTargetData();

            
        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Initialize Target Context
            // -------------------------------
            // (Function Overloading)
            /** \brief Initialize Target Context
            *
            * Uses the given target data to create class related functionalities
            *
            * \param target_data Target data [target_msgs::TargetData]
            */
            void init(const target_msgs::TargetData& target_data);


            // Initialize Target Context
            // -------------------------------
            // (Function Overloading)
            /** \brief Initialize Target Context
            *
            * Uses the given parameter name to load target parameter data from parameter server
            * Parameter data is then used to create the related target message-type
            * and class related functionalities
            *
            * \param param_name Target parameter name, located on parameter server [std::string]
            */
            void init(const std::string& param_name);


            // Initialize Target Context
            // -------------------------------
            // (Function Overloading)
            /** \brief Initialize Target Context
            *
            * Load the given parameter data located on the parameter server.
            * Parameter data is then used to create the class related target message-type
            * and class related functionalities
            *
            * \param param_xml  Target parameter, located on parameter server [XmlRpc::XmlRpcValue]
            */
            void init(const XmlRpc::XmlRpcValue& param_xml);


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


            // Load Target Joint Data
            // -------------------------------
            /** \brief Reads and loads information on custom target-joint from the parameter server.
            *
            * Organize and structure the loaded parameters into target-joint message-type.
            * If successful, the gathered target is returned.
            * Function returns fals if it fails to load target-joint data.
            *
            * \param param_xml  Target-Joint parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Successful: target-joint data [target_msgs::TargetJoint] / Unsuccessful: false [bool]
            */
            boost::optional<target_msgs::TargetJoint> loadParamJointData(
                const XmlRpc::XmlRpcValue& param_xml);


            // Load Target Cartesian Data
            // -------------------------------
            /** \brief Reads and loads information on custom target-cartesian from the parameter server.
            *
            * Organize and structure the loaded parameters into target-cartesian message-type.
            * If successful, the gathered target is returned.
            * Function returns fals if it fails to load target-cartesian data.
            *
            * \param param_xml  Target-Cartesian parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Successful: target-cartesian data [target_msgs::TargetCartesian] / Unsuccessful: false [bool]
            */
            boost::optional<target_msgs::TargetCartesian> loadParamCartesianData(
                const XmlRpc::XmlRpcValue& param_xml);


            // Print Target Joint Data
            // -------------------------------
            /** \brief Print information on target joint data to terminal.
            *
            * Implemented for debugging purposes.
            */
            void printTargetJointData();


            // Print Target Cartesian Data
            // -------------------------------
            /** \brief Print information on target cartesian data to terminal.
            *
            * Implemented for debugging purposes.
            */
            void printTargetCartesianData();


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
            std::map<std::string, TargetType> target_type_map_;

            // ROS Nodehandle(s)
            // -------------------------------
            ros::NodeHandle nh_;
            
            // ROS Publsiher(s)
            // -------------------------------
            ros::Publisher target_pub_;


            // Initialize Target Type Map
            // -------------------------------
            /** \brief Initialize target type map
            *
            * Each target-type [TargetType] is paired with a name [std::string],
            * where the entries of target-types matches the defined types in TargetData.msg
            * Function is aimed to initialzie the local target-type map [std::map<std::string, TargetType>]
            *
            * \return Target-Type Map [std::map<std::string, TargetType>]
            */
            static std::map<std::string, TargetType> initTargetTypeMap();

    }; // End Class: TargetContext
} // End Namespace: Info
#endif // TARGET_CONTEXT_H 