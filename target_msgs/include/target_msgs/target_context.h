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

    // Target
    #include "target_msgs/target_joint_context.h"
    #include "target_msgs/target_cartesian_context.h"

    // Target Messages
    #include "target_msgs/TargetJoint.h"
    #include "target_msgs/TargetCartesian.h"
    #include "target_msgs/TargetData.h"
    #include "target_msgs/TargetExtAxis.h"

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

    // // External Axis Type 
    // // (Matches the types defined in TargetExtAxis.msg)
    // enum ExtAxisType
    // {
    //     ROTATION,
    //     LINEAR
    // };

    // Target Context Class
    // -------------------------------
    /** \brief Robot system target context
    *
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

            // Load Target Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target from the parameter server.
            *
            * Organize and structure the loaded parameters into target message-type.
            * If successful, the gathered target is returned. 
            * If parameter loading fails, function returns false.
            *
            * \param param_name Target parameter name, located on parameter server [std::string]
            * \return Function return: Successful: Target data [target_msgs::TargetData] / Unsuccessful: false [bool]
            */
            boost::optional<target_msgs::TargetData> loadParamData(
                const std::string& param_name);


            // Load Target Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target from the parameter server.
            *
            * Organize and structure the loaded parameters into target message-type.
            * If successful, the gathered target is returned. 
            * If parameter loading fails, function returns false.
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
            target_msgs::TargetData target_data_;
            std::map<std::string, TargetType> target_type_map_;
            std::vector<std::string> target_type_names_vec_;

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
            * Function populates and returns the target-type map [std::map<std::string, TargetType>]
            *
            * \return Target-Type Map [std::map<std::string, TargetType>]
            */
            static std::map<std::string, TargetType> initTargetTypeMap();


            // Initialize Target Type Names
            // -------------------------------
            /** \brief Initialize target type names
            *
            * Each target-type [TargetType] is paired with a name [std::string] in a map.
            * Function iterates over the given target-type map and stores the target-type names in a vector.
            *
            * \param target_type_map Target-Type Map [std::map<std::string, TargetType>]
            * \return Target-Type Names [std::vector<std::string>]
            */
            static std::vector<std::string> initTargetTypeNames(
                std::map<std::string, TargetType> target_type_map);

    }; // End Class: TargetContext
} // End Namespace: Target
#endif // TARGET_CONTEXT_H 