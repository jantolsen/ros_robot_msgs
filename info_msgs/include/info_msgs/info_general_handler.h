// Info General Handler
// -------------------------------
// Description:
//      Robot system General information handler 
//      Collects information on system general parameters and configuration 
//      from the parameter server. It then structures and sorts the 
//      information into the info-general-message type and enables the 
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
#ifndef INFO_GENERAL_HANDLER_H       
#define INFO_GENERAL_HANDLER_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include <robot_toolbox/toolbox.h>

    // Info-Messages
    #include "info_msgs/InfoGeneral.h"

// Namespace: Info
// -------------------------------
namespace Info
{
    // Constants
    // -------------------------------
        

    // Enums
    // -------------------------------
        

    // Info-General-Handler Class
    // -------------------------------
    /** \brief Robot system General information handler 
    * Collects information on system general parameters and configuration 
    * from the parameter server. It then structures and sorts the 
    * information into the info-General-message type and enables the 
    * collected information to be published and shared
    */
    class InfoGeneralHandler
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Class constructor
            // -------------------------------
            /** \brief Info-General-Handler class constuctor
            */
            InfoGeneralHandler();


            // Class destructor
            // -------------------------------
            /** \brief Info-General-Handler class destructor
            */
            ~InfoGeneralHandler();


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Initialize Info-General-Handler
            // -------------------------------
            /** \brief Initialize Info-General-Handler
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
            * \param info_General Reference to Information-General [info_msgs::InfoGeneral]
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

            
        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;


    }; // End Class: InfoGeneralHandler
} // End Namespace: Info
#endif // INFO_GENERAL_HANDLER_H 