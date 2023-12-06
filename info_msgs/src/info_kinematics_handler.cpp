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
            info_kinematics.solver_id = static_cast<int>(param_xml["solver_type"]);
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
            XmlRpc::XmlRpcValue& param_xml)
        {
            // Reads the parameter-data obtained from parameter-server
            // and validates the data againt the info-message-type

            // Define local variable(s)
            std::string param_name;
            std::string param_solver_name;
            int param_solver_type;
            bool solver_name_fault = false;
            bool solver_type_fault = false;

            // Solver Name
            // -------------------------------
                // Target parameter
                param_name = "solver_name";

                // Check Parameter
                if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeString))
                {
                    // Raise local helper flag
                    solver_name_fault = true;

                    // Parameter validation failed
                    ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Warning! Parameter [" << param_name << "] is missing or configured incorrectly");
                    ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Trying to find data on related [type] and assign [name] accordingly");

                    // Revisit issue later with second approach:
                    //  1: Check solver-type parameter
                    //  2: Search type-map for solver-type and assign solver-name accordingly
                }
                // Parameter found
                else
                {
                    // Update local variable with obtained parameter data
                    param_solver_name = static_cast<std::string>(param_xml[param_name]);
                }

            // Solver Type
            // -------------------------------
                // Target parameter
                param_name = "solver_type";

                // Check Parameter
                if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeInt))
                {
                    // Raise local helper flag
                    solver_type_fault = true;

                    // Parameter validation failed
                    ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Warning! Parameter [" << param_name << "] is missing or configured incorrectly");
                    ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Trying to find data on related [name] and assign [type] accordingly");

                    // Revisit issue later with second approach:
                    //  1: Check solver-name parameter
                    //  2: Search type-map for solver-name and assign solver-ID accordingly
                }
                // Parameter found
                else
                {
                    // Update local variable with obtained parameter data
                    param_solver_type = static_cast<int>(param_xml[param_name]);
                }

            // Compare Solver Name vs Type
            // -------------------------------
                // Check fault flag on both Solver Name and Type
                if(solver_name_fault && solver_type_fault)
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameters [solver_type] and [solver_name] are missing or configured incorrectly");

                    // Function return
                    return false;
                }
                // Solver Name fault flag
                else if(solver_name_fault)
                {
                    // Find Solver Name in Type-Map using Solver Type
                    if(Toolbox::Parameter::searchTypeMap(param_solver_type, kinematicSolverTypeMap, param_solver_name)) 
                    {
                        // Assign solver-name equal to the located element of the type-map
                        param_xml["solver_name"] = param_solver_name;

                    }

                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameters [solver_type] and [solver_name] are missing or configured incorrectly");

                    // Function return
                    return false;
                }
                // Solver Type fault flag
                else if(solver_type_fault)
                {
                    // Find Solver Type in Type-Map using Solver Name
                    if(Toolbox::Parameter::searchTypeMap(param_solver_name, kinematicSolverTypeMap, param_solver_type)) 
                    {
                        // Assign solver-type equal to the located element of the type-map
                        param_xml["solver_type"] = param_solver_type;
                    }
                }
                // No fault flag
                // Check Solver Name against Solver Type
                else
                {
                    // Find Solver Name in Type-Map using Solver Type
                    if(!Toolbox::Parameter::searchTypeMap(param_solver_type, kinematicSolverTypeMap, param_solver_name))
                    {
                        
                    }
                }
    
            // Compare Solver-Name against Solver-Type

                // Parameter validation failed
                ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Warning! Parameter [" << param_name << "] is missing or configured incorrectly");
                ROS_WARN_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    <<  ": Trying to find data on [solver_type] and assign [solver_name] accordingly");

                // Trying second approach:
                //  1: Check solver-type parameter
                //  2: Search type-map for solver-type and assign solver-name accordingly
                param_name = "solver_id";

                // Check parameter
                if(!Toolbox::Parameter::checkParameter(param_xml, param_name, XmlRpc::XmlRpcValue::TypeString))
                {
                    // Parameter validation failed
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        <<  ": Failed! Parameters [solver_type] and [solver_name] are missing or configured incorrectly");

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
        } // Function End: validateParamInfoKinematics() 



} // End Namespace: Template