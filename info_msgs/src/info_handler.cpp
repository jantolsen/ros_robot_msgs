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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "info_msgs/info_handler.h"

// Namespace: Info
// -------------------------------
namespace Info
{
    // Info-Handler Class
    // -------------------------------

        // Constants
        // -------------------------------
        const std::string InfoHandler::CLASS_PREFIX = "InfoHandler::";


        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        InfoHandler::InfoHandler(
            ros::NodeHandle& nh)
        :
            nh_(nh)
        {
            // Initialize Info-Handler
            init();
        } // Class Constructor End: InfoHandler()


        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        InfoHandler::InfoHandler(
            ros::NodeHandlePtr& nhPtr)
        :
            // Constructor delegation
            InfoHandler(*nhPtr)
        {
            // This constructor delegates the construction of the Info-Handler-class to:
            // InfoHandler(ros::NodeHandler& nh)
        } // Class Constructor End: InfoHandler()


        // Class Desctructor
        // -------------------------------
        InfoHandler::~InfoHandler()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Destructor called");

        } // Class Desctructor End: ~InfoHandler()


        // Initialize Template-Class
        // -------------------------------
        void InfoHandler::init()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing class");
        } // Function End: init()

} // End Namespace: Template