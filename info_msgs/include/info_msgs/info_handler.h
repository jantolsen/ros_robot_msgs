// Info Handler
// -------------------------------
// Description:
//      Robot system information handler.
//      Collects information on system parameters and configuration 
//      from the parameter server. It then structures and sorts the 
//      information into the info-message types and enables the 
//      collected information to be published and shared 
//
// Version:
//  0.1 - Initial Version
//        [26.11.2023]  -   Jan T. Olsen
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
#ifndef INFO_HANDLER_H       
#define INFO_HANDLER_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Info-Messages
    #include "info_msgs/InfoGeneral.h"

// Namespace: Template
// -------------------------------
namespace Info
{
    // Enums
    // -------------------------------
        // Kinematic Solver Type 
        // (Matches the types defined in InfoKinematics.msg)
        enum KinematicSolver
        {
            KDL,        
            OPW,
            TRACIK,       
            LMA,  
            CACHED_KDL,
            CACHED_TRACIK
        };

    // Info-Handler Class
    // -------------------------------
    /** \brief Robot system information handler.
    * Collects information on system parameters and configuration 
    * from the parameter server. It then structures and sorts the 
    * information into the info-message types and enables the 
    * collected information to be published and shared 
    */
    class InfoHandler
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Info-Handler Constuctor
            * \param nh Reference to a ROS Nodehandle [ros::Nodehandle]
            */
            InfoHandler(
                ros::NodeHandle& nh);


            // Class constructor
            // -------------------------------
            // (Constructor Overloading)
            /** \brief Info-Handler Constuctor
            * \param nhPtr Reference to a ROS Nodehandle pointer [ros::NodeHandlePtr]
            */
            InfoHandler(
                ros::NodeHandlePtr& nhPtr);


            // Class destructor
            // -------------------------------
            /** \brief Template-Class destructor
            */
            ~InfoHandler();


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Initialize Info-Handler
            // -------------------------------
            /** \brief Initialize Info-Handler
            */
            void init();


            // Load Information-General Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads the Information-General from the parameter server.
            * Organize and structure the loaded parameters into the respective info-message-type
            * \param param_xml Information-General parameters [XmlRpc::XmlRpcValue]
            * \param info_general Reference to Information-General [info_msgs::InfoGeneral]
            */
            bool loadParamInfoGeneral(
                const XmlRpc::XmlRpcValue& param_xml,
                info_msgs::InfoGeneral& info_general);


            // Load Information-General Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads the Information-General from the parameter server.
            * Organize and structure the loaded parameters into the respective info-message-type
            * \param param_name Name of the Information-General parameters, located on parameter server [std::string]
            * \param info_general Reference to Information-General [info_msgs::InfoGeneral]
            */
            bool loadParamInfoGeneral(
                const std::string& param_name,
                info_msgs::InfoGeneral& info_general);


            // Validate Information-General Parameter Data
            // -------------------------------
            /** \brief Validate Information-General parameter data.
            * Parameters are obtained from XmlRpcValue data-type. Which acts as a generic collector.
            * This function runs through each element of the parameter data
            * and ensured its correctly configured according to the info-message-type
            * \param param_xml Information-General parameters [XmlRpc::XmlRpcValue]
            * \return Function result: Successful/Unsuccessful (true/false)
            */
            static bool validateParamInfoGeneral(
                const XmlRpc::XmlRpcValue& param_xml);


            // Load Information-Kinematics Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads the Information-Kinematics from the parameter server.
            * Organize and structure the loaded parameters into the respective info-message-type
            * \param param_xml Information-Kinematics parameters [XmlRpc::XmlRpcValue]
            * \param info_general Reference to Information-Kinematics [info_msgs::InfoKinematics]
            */
            bool loadParamInfoKinematics(
                const XmlRpc::XmlRpcValue& param_xml,
                info_msgs::InfoKinematics& info_kinematics);


            // Load Information-Kinematics Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads the Information-Kinematics from the parameter server.
            * Organize and structure the loaded parameters into the respective info-message-type
            * \param param_name Name of the Information-Kinematics parameters, located on parameter server [std::string]
            * \param info_kinematics Reference to Information-Kinematics [info_msgs::InfoKinematics]
            */
            bool loadParamInfoKinematics(
                const std::string& param_name,
                info_msgs::InfoKinematics& info_kinematics);


            // Validate Information-Kinematics Parameter Data
            // -------------------------------
            /** \brief Validate Information-Kinematics parameter data.
            * Parameters are obtained from XmlRpcValue data-type. Which acts as a generic collector.
            * This function runs through each element of the parameter data
            * and ensured its correctly configured according to the info-message-type
            * \param param_xml Information-Kinematics parameters [XmlRpc::XmlRpcValue]
            * \return Function result: Successful/Unsuccessful (true/false)
            */
            static bool validateParamInfoKinematics(
                const XmlRpc::XmlRpcValue& param_xml);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // ROS Nodehandle(s)
            ros::NodeHandle nh_;


    }; // End Class: InfoHandler
} // End Namespace: Info
#endif // INFO_HANDLER_H 