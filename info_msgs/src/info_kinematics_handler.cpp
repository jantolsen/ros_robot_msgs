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

            // // Initialize Kinematics-Solver-Type Map
            // initKinematicSolverTypeMap(kinematicSolverTypeMap_);

            // // Load Parameter
            // loadParamInfoKinematics(kinematics_param_name_, info_kinematics_data_);
        } // Function End: init()


        // Test Function
        // -------------------------------
        void InfoKinematicsHandler::test(
            std::string param, 
            info_msgs::InfoKinematics& info_kinematics_data)
        {
            // Initialize Kinematics-Solver-Type Map
            initKinematicSolverTypeMap(kinematicSolverTypeMap_);

            // Load Parameter
            loadParamInfoKinematics(param, info_kinematics_data);
        } // Function End: init()


        // Load Information-Kinematics Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoKinematicsHandler::loadParamInfoKinematics(
            const std::string& param_name,
            info_msgs::InfoKinematics& info_kinematics_data)
        {
            // Define local variable(s)
            XmlRpc::XmlRpcValue param_xml;
            
            // Check parameter server for Information-Kinematics parameters
            // if(!ros::param::get("/" + param_name, param_xml))
            if(!ros::param::get(param_name, param_xml))
            {
                // Failed to get parameter
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Failed! Info-Kinematics Parameter [" << param_name << "] not found");

                // Function return
                return false;
            }
            // Function return: Call overloading function
            return loadParamInfoKinematics(param_xml, info_kinematics_data);
        } // Function End: loadParamInfoKinematics() 


        // Load Information-Kinematics Parameter Data
        // -------------------------------
        // (Function Overloading)
        bool InfoKinematicsHandler::loadParamInfoKinematics(
            const XmlRpc::XmlRpcValue& param_xml,
            info_msgs::InfoKinematics& info_kinematics_data)
        {
            // Reads and loads parameter data obtained from the parameter-server
            // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
            // Elements of the loaded parameters are validated and assigned to the respective element in the info-message-type
            // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

            // Check if given parameter is a struct-type
            if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
            {
                // Parameter is not a struct
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Given Information-Kinematics parameter is not a struct");

                // Function return
                return false;
            }

            // Try to load parameters
            try
            {
                // Load, validate and assign parameter data
                info_kinematics_data.solver_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");
                info_kinematics_data.solver_type = Toolbox::Parameter::getParamData<int>(param_xml, "solver_type", kinematicSolverTypeMap_);
                info_kinematics_data.search_resolution = Toolbox::Parameter::getParamData<double>(param_xml, "search_resolution");
                info_kinematics_data.timeout = Toolbox::Parameter::getParamData<double>(param_xml, "timeout");
                info_kinematics_data.attempts = Toolbox::Parameter::getParamData<int>(param_xml, "attempts");
            }
            // Catch exception(s)
            catch (const std::exception& e) 
            {
                // Parameter loading failed
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter(s) related to Information-Kinematics" 
                    << " is either missing or configured incorrectly");

                // Exception details
                std::cerr << e.what() << std::endl;

                // Function return
                return false;
            } 

            // Function return
            return true;
        } // Function End: loadParamInfoKinematics() 


        // Update Info-Kinematic Data
        // -------------------------------
        void InfoKinematicsHandler::updateInfoKinematicsData(
            const info_msgs::InfoKinematics& info_kinematics_data)
        {
            // Update local Info-Kinematics Data
            info_kinematics_data_ = info_kinematics_data;
        } // Function End: updateInfoKinematicsData() 


        // Get Info-Kinematic Data
        // -------------------------------
        info_msgs::InfoKinematics InfoKinematicsHandler::getInfoKinematicsData()
        {
            // Return local Info-Kinematics Data
            return info_kinematics_data_;
        } // Function End: getInfoKinematicsData() 

        
        // Get Kinematic Solver Type Map
        // -------------------------------
        std::map<std::string, KinematicSolverType> InfoKinematicsHandler::getKinematicSolverTypeMap()
        {
            // Return local Kinematic-Solver-Type Map
            return kinematicSolverTypeMap_;
        } // Function End: initKinematicSolverTypeMap() 


        // Initialize Kinematic Solver Type Map
        // -------------------------------
        void InfoKinematicsHandler::initKinematicSolverTypeMap(
            std::map<std::string, KinematicSolverType>& kinematic_solver_type_map)
        {
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
        } // Function End: initKinematicSolverTypeMap() 

} // End Namespace: Info