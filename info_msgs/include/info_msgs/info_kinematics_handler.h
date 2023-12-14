// Info Kinematics Handler
// -------------------------------
// Description:
//      Robot system kinematics information handler 
//      Collects information on system kinematics parameters and configuration 
//      from the parameter server. It then structures and sorts the 
//      information into the info-kinematics-message type and enables the 
//      collected information to be published and shared
//
// Version:
//  0.1 - Initial Version
//        [06.12.2023]  -   Jan T. Olsen
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
#ifndef INFO_KINEMATICS_HANDLER_H       
#define INFO_KINEMATICS_HANDLER_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>
    #include <map>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include <robot_toolbox/toolbox.h>

    // Info-Messages
    #include "info_msgs/InfoKinematics.h"

// Namespace: Info
// -------------------------------
namespace Info
{
    // Enums
    // -------------------------------
        // Kinematic Solver Type 
        // (Matches the types defined in InfoKinematics.msg)
        enum KinematicSolverType
        {
            KDL,
            OPW,
            TRACIK,
            LMA,
            CACHED_KDL,
            CACHED_TRACIK
        };


    // Constants
    // -------------------------------
        


    // Info-Kinematics-Handler Class
    // -------------------------------
    /** \brief Robot system kinematics information handler 
    * Collects information on system kinematics parameters and configuration 
    * from the parameter server. It then structures and sorts the 
    * information into the info-kinematics-message type and enables the 
    * collected information to be published and shared
    */
    class InfoKinematicsHandler
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Class constructor
            // -------------------------------
            /** \brief Info-Kinematics-Handler class constuctor
            */
            InfoKinematicsHandler();


            // Class destructor
            // -------------------------------
            /** \brief Info-Kinematics-Handler class destructor
            */
            ~InfoKinematicsHandler();


            // Test Function
            // -------------------------------
            /** \brief Test Function
            */
            void test(
                std::string param, 
                info_msgs::InfoKinematics& info_kinematics);


            // Get Kinematic-Solver-Type Map
            // -------------------------------
            /** \brief Get the Kinematic-Solver-Type Map
            * Each solver-type [KinematicSolverType] is paired with a name (std::string),
            * where the entries of solver-types matches the defined types in InfoKinematics.msg.
            * \return Kinematic Solver Type Map [std::map<std::string, KinematicSolverType>]
            */
            std::map<std::string, KinematicSolverType> getKinematicSolverTypeMap();

        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Initialize Info-Kinematics-Handler
            // -------------------------------
            /** \brief Initialize Info-Kinematics-Handler
            */
            void init();


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


            // // Validate Information-Kinematics Parameter Data
            // // -------------------------------
            // /** \brief Validate Information-Kinematics parameter data.
            // * Parameters are obtained as XmlRpcValue data-type, Which acts as a generic collector.
            // * This function runs through each element of the parameter data
            // * and ensured its correctly configured according to the info-message-type
            // * \param param_xml Information-Kinematics parameters [XmlRpc::XmlRpcValue]
            // * \param info_kinematics Reference to Information-Kinematics [info_msgs::InfoKinematics]
            // * \return Function result: Successful/Unsuccessful (true/false)
            // */
            // void validateInfoKinematics(
            //     const XmlRpc::XmlRpcValue& param_xml,
            //     info_msgs::InfoKinematics& info_kinematics);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Kinematic Solver Type Map
            std::map<std::string, int> const kinematicSolverTypeMap_;
            
            // Initialize Kinematic Solver Type Map
            // -------------------------------
            /** \brief Initialize Kinematic Solver Type Map.
            * Each solver-type [KinematicSolverType] is paired with a name (std::string),
            * where the entries of solver-types matches the defined types in InfoKinematics.msg.
            * Function initializes the local Kinematic Solver Type Map [std::map<std::string, KinematicSolverType>]
            */
            std::map<std::string, KinematicSolverType> initKinematicSolverTypeMap();

            // Handle Error: Load Parameter Data
            // -------------------------------
            /** \brief Error-Handler when loading parameter data fails.
            * Function reports error message and throws exception.
            * \param param_name Name of parameter data that failed loading [std::string]
            */
            void handleErrorLoadParam(std::string param_name);

    }; // End Class: InfoKinematicsHandler
} // End Namespace: Info
#endif // INFO_KINEMATICS_HANDLER_H 