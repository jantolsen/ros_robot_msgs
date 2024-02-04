// Target Base
// -------------------------------
// Description:
//      Robot system target base interface.
//      Target base acts as base-class and interface for robot system targets.
//      The interface defines common behaviour and methods, to collect and structure
//      information on the custom target from the parameter server. 
// 
//      The target base interface is implemented as a part of the strategy-pattern
//      implementation of the robot-target. This interface defines the common behaviour and methods
//      for all strategies (target-types). The related strategies must then implement this interface
//      to provide their unique implementation of the behaviour and methods.
//
// Version:
//  0.2 -   Overhaul of Target implementation.
//          Introduding strategy-pattern.
//          Utilizing interface and target-types (strategies) into seperate classes.
//          [20.01.2024]  -   Jan T. Olsen
//  0.2 -   Overhaul of Target implementation.
//          Split target-type into seperate classes.
//          [16.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [09.01.2024]  -   Jan T. Olsen
//
// -------------------------------
// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "target_data/target_base.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Base Class
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetBase::CLASS_PREFIX = "TargetBase::";


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetBase::TargetBase(
        ros::NodeHandle& nh,
        const target_data::TargetData& target_data)
    :
        nh_(nh),
        target_data_(target_data)
    {

    } // Class Constructor End: TargetBase()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetBase::TargetBase(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        TargetBase(nh, loadParamData(param_name).value())
    {
        // This constructor delegates the construction of the TargetBase-class to:
        // TargetBase(ros::NodeHandler& nh, const target_data::TargetData& target_header)
    } // Class Constructor End: TargetBase()

    
    // // Class constructor
    // // -------------------------------
    // // (Constructor Overloading)
    // TargetBase::TargetBase(
    //     ros::NodeHandle& nh,
    //     const XmlRpc::XmlRpcValue& param_xml)
    // :
    //     TargetBase(nh, loadParamTargetData(param_name).value())
    // {
    //     // This constructor delegates the construction of the TargetBase-class to:
    //     // TargetBase(ros::NodeHandler& nh, const target_data::TargetData& target_header)
    // } // Class Constructor End: TargetBase()


    // Class Desctructor
    // -------------------------------
    TargetBase::~TargetBase()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");
    } // Class Desctructor End: ~TargetBase()
    

    // Get This
    // -------------------------------
    TargetBase* TargetBase::getThis()
    {
        // Return a pointer to the current instance of this class
        return this;
    } // Function End: getThis()


    // Get Target Data
    // -------------------------------
    target_data::TargetData TargetBase::getTargetData()
    {
        // Return local target data
        return target_data_;
    } // Function End: getTargetData()


    // Get Target-Type Map
    // -------------------------------
    std::map<std::string, TargetType> TargetBase::getTargetTypeMap()
    {
        // Return local target-type map
        return target_type_map_;
    } // Function End: getTargetTypeMap()


    // Get Target-Type Names
    // -------------------------------
    std::vector<std::string> TargetBase::getTargetTypeNames()
    {
        // Return local target-type names vector
        return target_type_names_vec_;
    } // Function End: getTargetTypeNames()


    // Update Target Data
    // -------------------------------
    void TargetBase::updateTargetData(
        target_data::TargetData target_data)
    {
        // Update local target data
        target_data_ = target_data;
    } // Function End: updateTargetData()


    // Load Target Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_data::TargetData> TargetBase::loadParamData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for Target parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                <<  ": Failed! Target Parameter [" << param_name << "] is NOT found");

            // Function return
            return boost::none;
        }
        // Function return
        return loadParamData(param_xml);
    } // Function End: loadParamData()


    // Load Target Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_data::TargetData> TargetBase::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

        // Define local variable(s)
        target_data::TargetData target_data;

        // Check if given parameter is a struct-type
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Target parameter is NOT a struct");

            // Function return
            return boost::none;
        }

        // Load, validate and assign parameter data
        try
        {
            // Get and load Target-Header parameter data
            target_data.header = getParamTargetHeader(param_xml);
        }
        // Catch Exception(s)
        catch (const std::exception& e) 
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to Target [" << target_data.header.name << "]" 
                << " is either missing or configured incorrectly");

            // Exception details
            std::cerr << e.what() << std::endl;

            // Function return
            return boost::none;
        } 
        // Function return
        return target_data;
    } // Function End: loadParamData()


    // Print Target-Header
    // -------------------------------
    void TargetBase::printTargetHeader()
    {
        // Print information on local target data to terminal
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("Target:");
        ROS_INFO_STREAM("--------------------");
        ROS_INFO_STREAM("Name: " << target_data_.header.name);
        ROS_INFO_STREAM("Type: " << target_data_.header.type_name); 
        
    } // Function End: printTargetHeader()


    // Get Target-Header Parameter Data
    // -------------------------------
    // (Function Overloading)
    target_data::TargetHeader TargetBase::getParamTargetHeader(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        target_data::TargetHeader target_header;
        std::map<std::string, TargetType> target_type_map = initTargetTypeMap();
        std::vector<std::string> target_type_names =  initTargetTypeNames(target_type_map);

        // Load, validate and assign parameter data
        target_header.name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");
        target_header.type_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "type", target_type_names);
        target_header.type = Toolbox::Parameter::getParamData<int>(param_xml, "type", target_type_map);

        // Function return
        return target_header;
    } // Function End: getParamTargetHeader()


    // Initialize Target Type Map
    // -------------------------------
    std::map<std::string, TargetType> TargetBase::initTargetTypeMap()
    {
        // Define and populate map
        std::map<std::string, TargetType> target_type_map =
        {
            {"JOINT",       TargetType::JOINT},
            {"CARTESIAN",   TargetType::CARTESIAN}
        };
        // Function return
        return target_type_map;
    } // Function End: initTargetTypeMap()


    // Initialize Target Type Names
    // -------------------------------
    std::vector<std::string> TargetBase::initTargetTypeNames(
        std::map<std::string, TargetType> target_type_map)
    {
        // Define vector
        std::vector<std::string> target_type_names_vec;

        // Iterate through target-type map
        for (auto& target_type : target_type_map)
        {
            // Append target-type name found within target-type map
            target_type_names_vec.push_back(target_type.first);
        }
        // Function return
        return target_type_names_vec;    
    } // Function End: initTargetTypeNames()

} // End Namespace: Target