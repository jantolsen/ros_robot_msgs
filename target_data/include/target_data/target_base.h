// Target Base
// -------------------------------
// Description:
//      Robot system target base interface.
//      Target base acts as base-class and interface for robot system targets.
//      The interface defines common behaviour and methods, to collect and structure
//      information on the custom target from the parameter server. 
// 
//      The target base interface is implemented as a part of the strategy-pattern
//      implementation of the robot-target. This interface defines the common behaviour and methods
//      for all strategies (target-types). The related strategies must then implement this interface
//      to provide their unique implementation of the behaviour and methods.
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
#ifndef TARGET_BASE_H       
#define TARGET_BASE_H

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
    #include "target_data/TargetHeader.h"
    #include "target_data/TargetData.h"
    #include "target_data/TargetJoint.h"
    #include "target_data/TargetCartesian.h"
    #include "target_data/TargetJointExtAxis.h"


// Namespace: Target
// -------------------------------
namespace Target
{
    // Enums
    // -------------------------------
    // Target Type 
    // (Matches the types defined in TargetHeader.msg)
    enum TargetType
    {
        JOINT,
        CARTESIAN,
        JOINT_EXTAXIS
    };


    // Target Base Class
    // -------------------------------
    /** \brief Robot system target base interface
    *
    * Target base acts as base-class and interface for robot system targets.
    * The interface defines common behaviour and methods, to collect and structure
    * information on the custom target from the parameter server. 
    * 
    * The target base interface is implemented as a part of the strategy-pattern
    * implementation of the robot-target. This interface defines the common behaviour and methods
    * for all strategies (target-types). The related strategies must then implement this interface
    * to provide their unique implementation of the behaviour and methods.
    */
    class TargetBase
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Define Shared-Pointer of Class-Object
            typedef typename std::shared_ptr<TargetBase> Ptr;

            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Base class constuctor
            *
            * \param nh                 ROS Nodehandle [ros::Nodehandle]
            * \param target_data        Target data [target_data::TargetData]
            */
            TargetBase(
                ros::NodeHandle& nh,
                const target_data::TargetData& target_data);


            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Base class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_name Target parameter name, located on parameter server [std::string]
            */
            TargetBase(
                ros::NodeHandle& nh,
                const std::string& param_name);

            
            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Base class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_xml  Target parameter, located on parameter server [XmlRpc::XmlRpcValue]
            */
            TargetBase(
                ros::NodeHandle& nh,
                const XmlRpc::XmlRpcValue& param_xml);


            // Class destructor
            // -------------------------------
            /** \brief Target-Base class destructor
            */
            virtual ~TargetBase();


            // Get This
            // -------------------------------
            /** \brief Get Target-Base object pointer
            *
            * \return Pointer to Target-Base object [TargetBase*]
            */
            virtual TargetBase* getThis();


            // Get Target Data
            // -------------------------------
            /** \brief Get collective information on custom target data.
            *
            * Target data contains collective information, parameters and configuration 
            * for the custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target-Data [target_data::TargetData]
            */
            virtual target_data::TargetData getTargetData();


            // Get Target Header Data
            // -------------------------------
            /** \brief Get information on custom target-header data.
            *
            * Target-Header data contains meta-data  on the custom defined target.
            * Typically information on target-name and -type
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target-Header data [target_data::TargetHeader]
            */
            virtual target_data::TargetHeader getTargetHeader();


            // Get Target Joint Data
            // -------------------------------
            /** \brief Get information on custom target-joint data.
            *
            * Target-Joint contains information, parameters and configuration
            * the for a joint-type custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target-Joint data [target_data::TargetJoint]
            */
            virtual target_data::TargetJoint getTargetJoint();


            // Get Target Cartesian Data
            // -------------------------------
            /** \brief Get information on custom target-cartesian data.
            *
            * Target-Cartesian contains information, parameters and configuration
            * the for a cartesian-type custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target-Cartesian data [target_data::TargetCartesian]
            */
            virtual target_data::TargetCartesian getTargetCartesian();


            // Get Target Joint External-Axis Data
            // -------------------------------
            /** \brief Get information on custom target-joint external-axis data.
            *
            * Target-Joint-ExtAxis contains information, parameters and configuration
            * the for a joint-extaxis-type custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Function return: Target-Joint data [target_data::TargetJointExtAxis]
            */
            virtual target_data::TargetJointExtAxis getTargetJointExtAxis();


            // Get Target-Type Map
            // -------------------------------
            /** \brief Get Target-Type Map.
            *
            * Each target-type [TargetType] is paired with a name [std::string],
            * where the entries of target-types matches the defined types in TargetHeader.msg
            *
            * \return Function return: Target-Type Map [std::map<std::string, TargetType>]
            */
            virtual std::map<std::string, TargetType> getTargetTypeMap();


            // Get Target-Type Names
            // -------------------------------
            /** \brief Get Target-Type Names.
            *
            * Each target-type [TargetType] is paired with a name [std::string] in a map.
            * Function iterates over the target-type map and collects the target-type names in a vector.
            *
            * \return Function return: Target-Type Names [std::vector<std::string>]
            */
            virtual std::vector<std::string> getTargetTypeNames();


            // Update Target Data
            // -------------------------------
            /** \brief Update custom target data information.
            *
            * Target data contains collective information, parameters and configuration 
            * for the custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \param target_data Updated Target-Data [target_data::TargetData]
            */
            virtual void updateTargetData(
                target_data::TargetData target_data);


            // Print Target Data
            // -------------------------------
            /** \brief Print information on target data to terminal.
            *
            * Implemented for debugging purposes.
            */
            virtual void printTargetData();


            // Get Target Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target from the parameter server.
            *
            * Organize and structure the loaded parameters into target message-type.
            * If successful, the gathered target-data is returned. 
            * If parameter loading fails, an error message is given a run-time exception is thrown.
            *
            * \param param_name Target parameter name, located on parameter server [std::string]
            * \return           Target-Data [target_data::TargetData]
            * \exception        Throws a run-time exception if paramter is not found or invalid target-type.  
            */
            // virtual target_data::TargetData getParamTargetData(
            target_data::TargetData getParamTargetData(
                const std::string& param_name);


            // Get Target Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom target from the parameter server.
            *
            * Organize and structure the loaded parameters into target message-type.
            * If successful, the gathered target-data is returned. 
            * If parameter loading fails, an error message is given a run-time exception is thrown.
            *
            * \param param_xml  Target parameters [XmlRpc::XmlRpcValue]
            * \return           Target-Data [target_data::TargetData]
            * \exception        Throws a run-time exception if paramter is not found or invalid target-type. 
            */
            // virtual target_data::TargetData getParamTargetData(
            target_data::TargetData getParamTargetData(
                const XmlRpc::XmlRpcValue& param_xml);

            
            // Get Target-Type Parameter
            // -------------------------------
            // (Function Overloading)
            /** \brief Get target-type related to the custom target from the parameter server.
            *
            * If the target-type parameter is found, the obtained value is validated against 
            * pre-defined valid target-types (target-type-map). 
            * 
            * Failing to load the parameter or unsuccessful validation, the function gives
            * an error message and a run-time exception is thrown.
            *     
            * \param param_name Target parameter name, located on parameter server [std::string]
            * \return           Function return: Target-Type [TargetType]
            * \exception        Throws a run-time exception if paramter is not found or invalid target-type. 
            */
            static TargetType getParamTargetType(
                const std::string& param_name);


            // Get Target-Type Parameter
            // -------------------------------
            // (Function Overloading)
            /** \brief Get target-type related to the custom target from the parameter server.
            *
            * If the target-type parameter is found, the obtained value is validated against 
            * pre-defined valid target-types (target-type-map). 
            * 
            * Failing to load the parameter or unsuccessful validation, the function gives
            * an error message and a run-time exception is thrown.
            *     
            * \param param_xml  Target parameter [XmlRpc::XmlRpcValue]
            * \return           Target-Type data [TargetType]
            * \exception        Throws a run-time exception if paramter is not found or invalid target-type. 
            */
            static TargetType getParamTargetType(
                const XmlRpc::XmlRpcValue& param_xml);


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Class Local Member(s)
            // -------------------------------
            target_data::TargetData target_data_;
            std::map<std::string, TargetType> target_type_map_;
            std::vector<std::string> target_type_names_vec_;

            // ROS Nodehandle(s)
            // -------------------------------
            ros::NodeHandle nh_;
            
            // ROS Publsiher(s)
            // -------------------------------
            ros::Publisher target_pub_;


            // Get Target-Header Parameter Data
            // -------------------------------
            /** \brief Reads and loads target-header information on custom target-header from the parameter server.
            *
            * Organize and structure the loaded parameters into target-header message-type.
            * If successful, the gathered target-header data is returned. 
            * If parameter loading fails, an error message is given and a runtime expection is thrown.
            *
            * \param param_xml  Target parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Target-Header data [target_data::TargetHeader]
            */
            target_data::TargetHeader getParamTargetHeader(
                const XmlRpc::XmlRpcValue& param_xml);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:

            // Initialize Target Type Map
            // -------------------------------
            /** \brief Initialize target type map
            *
            * Each target-type [TargetType] is paired with a name [std::string],
            * where the entries of target-types matches the defined types in TargetHeader.msg
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

    }; // End Class: TargetBase
} // End Namespace: Target
#endif // TARGET_BASE_H 