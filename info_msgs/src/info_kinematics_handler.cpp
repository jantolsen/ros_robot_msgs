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
//        [26.11.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "info_msgs/info_kinematics_handler.h"

// Namespace: Info
// -------------------------------
namespace Info
{
    // Info-Kinematics Class
    // -------------------------------

        // Constants
        // -------------------------------
        const std::string InfoKinematicsHandler::CLASS_PREFIX = "InfoKinematicsHandler::";


        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        InfoKinematicsHandler::InfoKinematicsHandler()
        {
            // Initialize Info-Kinematics-Handler
            init();
        } // Class Constructor End: InfoKinematicsHandler()


        // Class Desctructor
        // -------------------------------
        InfoKinematicsHandler::~InfoKinematicsHandler()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Destructor called");

        } // Class Desctructor End: ~InfoKinematicsHandler()


        // Initialize Info-Kinematics-Handler
        // -------------------------------
        void InfoKinematicsHandler::init()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing class");
        } // Function End: init()


        // Load Information-Kinematics Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoKinematicsHandler::loadParamInfoKinematics(
            const XmlRpc::XmlRpcValue& param_xml,
            info_msgs::InfoKinematics& info_kinematics)
        {
            // Reads and loads data from obtained parameters from the parameter-server
            // Parameters are obtained from XmlRpcValue data-type which acts as generic collector.
            // Elements of the loaded parameters are assigned to the respective element in the info-message-type
            // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

            // Validate Information-Kinematics Parameters
            // -------------------------------
            if(!validateParamInfoKinematics(param_xml))
            {
                // Validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Parameters validation failed!");

                // Function return
                return false;
            }

            // Get Solver-Type data
            // -------------------------------
            // Solver-type can be configured by either the solver-name [string] or solver-type [int] on the parameter-server
            // Temporary variables
            int solver_type;
            std::string solver_name;
            const std::string param_name = static_cast<std::string>(param_xml["solver_type"]);

            // Get Type-Item data
            // (using the solver-type parameter information, find the related item (solver-name or solver-type) 
            // from the defined KinematicSolverTypeMap)
            Toolbox::Parameter::getTypeItem(param_xml, param_name, kinematicSolverTypeMap, solver_type, solver_name);

            // Parameter data assignment
            // -------------------------------
            info_kinematics.solver_type = solver_type;
            info_kinematics.solver_name = solver_name;
            info_kinematics.search_resolution = static_cast<double>(param_xml["search_resolution"]);
            info_kinematics.timeout = static_cast<double>(param_xml["timeout"]);
            info_kinematics.attempts = static_cast<double>(param_xml["attempts"]);

            // Function return
            return true;
        } // Function End: loadParamInfoKinematics() 


        // Load Information-Kinematics Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoKinematicsHandler::loadParamInfoKinematics(
            const std::string& param_name,
            info_msgs::InfoKinematics& info_kinematics)
        {
            // Define local variable(s)
            XmlRpc::XmlRpcValue param_xml;
            
            // Check parameter server for Information-Kinematics parameters
            if(!ros::param::get("/" + param_name, param_xml))
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
        bool InfoKinematicsHandler::validateParamInfoKinematics(
            const XmlRpc::XmlRpcValue& param_xml)
        {
            // Reads the parameter-data obtained from parameter-server
            // and validates the data againt the info-message-type

            // Define local variable(s)
            std::string param_name;

            // Solver Type
            // -------------------------------
                // Target parameter
                // (can be configured as either type [int] or name [std::string] on the parameter-server)
                param_name = "solver_type";

                // Check type-item parameter
                if(!Toolbox::Parameter::checkTypeItem(param_xml, param_name, kinematicSolverTypeMap))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameter [" << param_name << "] is missing or configured incorrectly");

                    // Function return
                    return false;
                }

            // Solver Search-Resolution
            // -------------------------------
                // Target parameter
                param_name = "search_resolution";

                // Check parameter
                if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeDouble))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameter [" << param_name <<"] is missing or configured incorrectly");

                    // Function return
                    return false;
                }

            // Solver Timeout
            // -------------------------------
                // Target parameter
                param_name = "timeout";

                // Check parameter
                if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeDouble))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameter [" << param_name <<"] is missing or configured incorrectly");

                    // Function return
                    return false;
                }

            // Solver Attempts
            // -------------------------------
                // Target parameter
                param_name = "attempts";

                // Check parameter
                if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeInt))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameter [" << param_name <<"] is missing or configured incorrectly");

                    // Function return
                    return false;
                }

            // Function return
            return true;
        } // Function End: validateParamInfoKinematics() 

} // End Namespace: Template