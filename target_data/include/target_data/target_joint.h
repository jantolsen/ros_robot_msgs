// Target Joint
// -------------------------------
// Description:
//      Robot system target joint.
//      Target joint provides behaviour and methods related to a custom target joint type.
//      Collects information of custom target data from the parameter server
//      Structures and sorts the gathered information to respective target-data message-type.
//      Providing functionalities to interact with and broadcast related target-data
//
//      The target joint is implemented as a part of the strategy-pattern implementation of the 
//      robot-target. This strategy implements the interface (target type interface) and provides 
//      unique implementation of the behaviour and methods related to target type.
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
#ifndef TARGET_JOINT_H       
#define TARGET_JOINT_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>
    
    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Target
    #include "target_data/target_base.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Base Class
    // -------------------------------
    /** \brief Robot system target joint.
    *
    * Target joint provides behaviour and methods related to a custom target joint type.
    * Collects information of custom target data from the parameter server
    * Structures and sorts the gathered information to respective target-data message-type.
    * Providing functionalities to interact with and broadcast related target-data
    *
    * The target joint is implemented as a part of the strategy-pattern implementation of the 
    * robot-target. This strategy implements the interface (target type interface) and provides 
    * unique implementation of the behaviour and methods related to target type.
    */
    class TargetJoint : public TargetBase
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Joint class constuctor
            *
            * \param nh                 ROS Nodehandle [ros::Nodehandle]
            * \param target_data        Target data [target_data::TargetData]
            */
            TargetJoint(
                ros::NodeHandle& nh,
                const target_data::TargetData& target_data);


            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Joint class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_name Target parameter name, located on parameter server [std::string]
            */
            TargetJoint(
                ros::NodeHandle& nh,
                const std::string& param_name);

            
            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Target-Joint class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_xml  Target parameter, located on parameter server [XmlRpc::XmlRpcValue]
            */
            TargetJoint(
                ros::NodeHandle& nh,
                const XmlRpc::XmlRpcValue& param_xml);


            // Get This
            // -------------------------------
            /** \brief Get Target-Joint object pointer
            *
            * \return Pointer to Target-Joint object [TargetJoint*]
            */
            virtual TargetJoint* getThis();


            // // Print Target Data
            // // -------------------------------
            // /** \brief Print information on target data to terminal.
            // *
            // * Implemented for debugging purposes.
            // */
            // void printTargetData() override;

            
            // // Get Target Parameter Data
            // // -------------------------------
            // /** \brief Reads and loads information on custom target from the parameter server.
            // *
            // * Organize and structure the loaded parameters into target message-type.
            // * If successful, the gathered target-data is returned. 
            // * If parameter loading fails, an error message is given a run-time exception is thrown.
            // *
            // * \param param_xml  Target parameters [XmlRpc::XmlRpcValue]
            // * \return           Target-Data [target_data::TargetData]
            // * \exception        Throws a run-time exception if paramter is not found or invalid target-type. 
            // */
            // target_data::TargetData getParamTargetData(
            //     const XmlRpc::XmlRpcValue& param_xml) override;


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;
            
            // Class Local Member(s)
            // -------------------------------
            target_data::TargetJoint target_joint_;


            // Get Target Joint Parameter Data
            // -------------------------------
            /** \brief Reads and loads target-joint information on custom target from the parameter server.
            *
            * Organize and structure the loaded parameters into target-joint message-type.
            * If successful, the gathered target-joint data is returned. 
            * If parameter loading fails, an error message is given and a runtime expection is thrown.
            *
            * \param param_xml  Target parameters [XmlRpc::XmlRpcValue]
            * \return Function return: Target-Joint data [target_data::TargetJoint]
            */
            target_data::TargetJoint getParamTargetJoint(
                const XmlRpc::XmlRpcValue& param_xml);


            // Print Target Joint
            // -------------------------------
            /** \brief Print information on target Joint to terminal.
            *
            * Implemented for debugging purposes.
            */
            void printTargetJoint();


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:


    }; // End Class: TargetJoint
} // End Namespace: Target
#endif // TARGET_JOINT_H 