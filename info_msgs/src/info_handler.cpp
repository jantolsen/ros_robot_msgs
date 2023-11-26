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


        // Load Information-Kinematics Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoHandler::loadParamInfoKinematics(
            const XmlRpc::XmlRpcValue& param_xml,
            info_msgs::InfoKinematics& info_kinematics)
        {
            // Reads and loads data from obtained parameters from parameter-server
            // Parameters are obtained from XmlRpcValue data-type which acts as generic collector.
            // Elements of the loaded parameters are assigned to the respective element in the info-message-type
            // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

            // Validate Information-Kinematics Parameters
            // -------------------------------
            if(!validateParamInfoKinematics(param_xml))
            {
                // Validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Info-Kinematics Parameters validation failed!");

                // Function return
                return false;
            }

            // Parameter data assignment
            // -------------------------------
            info_kinematics.solver_name = static_cast<std::string>(param_xml["solver_name"]);
            info_kinematics.solver_id = static_cast<int>(param_xml["solver_id"]);
            info_kinematics.search_resolution = static_cast<double>(param_xml["search_resolution"]);
            info_kinematics.timeout = static_cast<double>(param_xml["timeout"]);
            info_kinematics.attempts = static_cast<double>(param_xml["attempts"]);

            // Function return
            return true;
        } // Function End: loadParamInfoKinematics() 


        // Load Information-Kinematics Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoHandler::loadParamInfoKinematics(
            const std::string& param_name,
            info_msgs::InfoKinematics& info_kinematics)
        {
            // Define local variables
            XmlRpc::XmlRpcValue param_xml;
            
            // Check parameter server for Information-Kinematics parameters
            if(!nh_.getParam("/" + param_name, param_xml))
            {
                // Failed to get parameter
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Failed! Info-Kinematics Parameter [" << param_name <<"] not found");

                // Function return
                return false;
            }
            // Function return: Call overloading function
            return loadParamInfoKinematics(param_xml, info_kinematics);
        } // Function End: loadParamInfoKinematics() 


        // Validate Information-Kinematics Parameter Data
        // -------------------------------
        bool InfoHandler::validateParamInfoKinematics(
            const XmlRpc::XmlRpcValue& param_xml)
        {
            // Function return
            return true;
        } // Function End: validateParamInfoKinematics() 



} // End Namespace: Template