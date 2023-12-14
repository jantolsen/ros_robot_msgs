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

            // Initialize Kinematics-Solver-Type Map
            kinematicSolverTypeMap_ = initKinematicSolverTypeMap();
        } // Function End: init()


        // Test Function
        // -------------------------------
        void InfoKinematicsHandler::test(
            std::string param, 
            info_msgs::InfoKinematics& info_kinematics)
        {
            // Initialize Kinematics-Solver-Type Map
            kinematicSolverTypeMap_ = initKinematicSolverTypeMap();

            // Load Parameter
            loadParamInfoKinematics(param, info_kinematics);
        } // Function End: init()


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
            // if(!ros::param::get(param_name, param_xml))
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


        // Load Information-Kinematics Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoKinematicsHandler::loadParamInfoKinematics(
            const XmlRpc::XmlRpcValue& param_xml,
            info_msgs::InfoKinematics& info_kinematics)
        {
            // Reads and loads parameter data obtained from the parameter-server
            // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
            // Elements of the loaded parameters are validated and assigned to the respective element in the info-message-type
            // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

            // Load, validate and assign parameter data
            // -------------------------------
            info_kinematics.solver_name = Toolbox::Parameter::getParamBool(param_xml, "solver_type").value_or(handleErrorLoadParam("solver_type"));
            info_kinematics.solver_type = Toolbox::Parameter::searchTypeMapByName(kinematicSolverTypeMap_, "solver_type").value_or(handleErrorLoadParam("solver_type"));
            info_kinematics.search_resolution = Toolbox::Parameter::getParamDouble(param_xml, "search_resolution").value_or(handleErrorLoadParam("search_resolution"));
            info_kinematics.timeout = Toolbox::Parameter::getParamDouble(param_xml, "timeout").value_or(handleErrorLoadParam("timeout"));
            info_kinematics.attempts = Toolbox::Parameter::getParamInt(param_xml, "attempts").value_or(handleErrorLoadParam("attempts"));

            // Function return
            return true;
        } // Function End: loadParamInfoKinematics() 


        // Get Kinematic Solver Type Map
        // -------------------------------
        std::map<std::string, KinematicSolverType> InfoKinematicsHandler::getKinematicSolverTypeMap()
        {
            // Return Kinematic-Solver-Type Map
            return kinematicSolverTypeMap_;
        } // Function End: initKinematicSolverTypeMap() 


        // Initialize Kinematic Solver Type Map
        // -------------------------------
        std::map<std::string, KinematicSolverType> InfoKinematicsHandler::initKinematicSolverTypeMap()
        {
            // Define local Kinematic-Solver-Type Map
            std::map<std::string, KinematicSolverType> kinematic_solver_type_map;
            
            // Initialize and populate map
            kinematic_solver_type_map =
            {
                {"KDL",             KinematicSolverType::KDL},
                {"OPW",             KinematicSolverType::OPW},
                {"TRACIK",          KinematicSolverType::TRACIK},
                {"LMA",             KinematicSolverType::LMA},
                {"CACHED_KDL",      KinematicSolverType::CACHED_KDL},
                {"CACHED_TRACIK",   KinematicSolverType::CACHED_TRACIK}
            };

            // Function return
            return kinematic_solver_type_map;
        } // Function End: initKinematicSolverTypeMap() 


        // Handle Error: Load Parameter Data
        // -------------------------------
        void InfoKinematicsHandler::handleErrorLoadParam(std::string param_name)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter [" << param_name <<"] is missing or configured incorrectly");
            
            // Throw exception
            throw std::runtime_error("Failed to get parameter [" + param_name +"]");
        } // Function End: handleErrorLoadParamData()

} // End Namespace: Template