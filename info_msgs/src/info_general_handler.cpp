// Info General Handler
// -------------------------------
// Description:
//      Robot system general information handler 
//      Collects information on system general parameters and configuration 
//      from the parameter server. It then structures and sorts the 
//      information into the info-general-message type and enables the 
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
    #include "info_msgs/info_general_handler.h"

// Namespace: Info
// -------------------------------
namespace Info
{
    // Info-General Class
    // -------------------------------

        // Constants
        // -------------------------------
        const std::string InfoGeneralHandler::CLASS_PREFIX = "InfoGeneralHandler::";


        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        InfoGeneralHandler::InfoGeneralHandler()
        {
            // Initialize Info-General-Handler
            init();
        } // Class Constructor End: InfoGeneralHandler()


        // Class Desctructor
        // -------------------------------
        InfoGeneralHandler::~InfoGeneralHandler()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Destructor called");

        } // Class Desctructor End: ~InfoGeneralHandler()


        // Initialize Info-General-Handler
        // -------------------------------
        void InfoGeneralHandler::init()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing class");
        } // Function End: init()


        // Load Information-General Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoGeneralHandler::loadParamInfoGeneral(
            const XmlRpc::XmlRpcValue& param_xml,
            info_msgs::InfoGeneral& info_general)
        {
            // Reads and loads data from obtained parameters from parameter-server
            // Parameters are obtained from XmlRpcValue data-type which acts as generic collector.
            // Elements of the loaded parameters are assigned to the respective element in the info-message-type
            // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

            // Validate Information-Kinematics Parameters
            // -------------------------------
            if(!validateParamInfoGeneral(param_xml))
            {
                // Validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Info-Kinematics Parameters validation failed!");

                // Function return
                return false;
            }

            // Parameter data assignment
            // -------------------------------

            // Function return
            return true;
        } // Function End: loadParamInfoGeneral() 


        // Load Information-General Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoGeneralHandler::loadParamInfoGeneral(
            const std::string& param_name,
            info_msgs::InfoGeneral& info_general)
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
        } // Function End: loadParamInfoGeneral() 


        // Validate Information-General Parameter Data
        // -------------------------------
        bool InfoGeneralHandler::validateParamInfoGeneral(
            const XmlRpc::XmlRpcValue& param_xml)
        {
            // Reads the parameter-data obtained from parameter-server
            // and validates the data againt the info-message-type

            // Define local variable(s)
            std::string param_name;

            // Solver Name
            // -------------------------------
            param_name = "solver_name";
            if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeString))
            {
                // Parameter validation failed
                ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Warning! Parameter [" << param_name << "] is missing or configured incorrectly");
                ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Trying to find data on [solver_id] and assign [solver_name] accordingly");

                // Try second approach of obtaining data for solver-name
                // Search for solver-id and assign solver-name accordingly
                param_name = "solver_id";
                if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeString))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameters [solver_id] and [solver_name] are missing or configured incorrectly");

                    // Function return
                    return false;
                }

                // Use solver-id to set solver-name parameter
                // -------------------------------
                // Get solver-id from parameter
                int solver_id = static_cast<int>(param_xml["solver_id"]);
                // Cast solver-id to kinematic-solver enum-type
                KinematicSolver kinematic_solver = static_cast<KinematicSolver>(solver_id);
                // Search for solver-name in kinematic-solver-map using solver-id as key
                auto search = KinematicSolverMap.find(kinematic_solver);

                // Check if searched element is found in the container
                if(search == KinematicSolverMap.end())
                {
                    // No element was found in the container
                    // (iterator has reached the end of the container)

                    // Report error to terminal
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameter [solver-id = " << solver_id << "] is not a valid Kinematic Solver Type");

                    // Function return
                    return false;
                }

                // Assign solver-name equal to the located element of the kinematic-solver-map
                param_xml["solver_name"] = search->second;
            }

            // Solver ID
            // -------------------------------
            param_name = "solver_id";
            if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeString))
            {
                // Parameter validation failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Failed! Parameter [" << param_name <<"] is missing or configured incorrectly");

                // Function return
                return false;
            }

            // Function return
            return true;
        } // Function End: validateParamInfoGeneral() 



} // End Namespace: Template