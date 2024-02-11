// Target Cartesian
// -------------------------------
// Description:
//      Robot system target cartesian.
//      Target cartesian provides behaviour and methods related to a custom target cartesian type.
//      Collects information of custom target data from the parameter server
//      Structures and sorts the gathered information to respective target-data message-type.
//      Providing functionalities to interact with and broadcast related target-data
//
//      The target cartesian is implemented as a part of the strategy-pattern implementation of the 
//      robot-target. This strategy implements the interface (target type interface) and provides 
//      unique implementation of the behaviour and methods related to target type.
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
    #include "target_data/target_cartesian.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Cartesian Class
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetCartesian::CLASS_PREFIX = "TargetCartesian::";


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetCartesian::TargetCartesian(
        ros::NodeHandle& nh,
        const target_data::TargetData& target_data)
    :
        TargetBase(nh, target_data)
    {

    } // Class Constructor End: TargetBase()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetCartesian::TargetCartesian(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        TargetBase(nh, getParamTargetData(param_name))
    {
        // This constructor delegates the construction of the TargetCartesian-class to:
        // TargetCartesian(ros::NodeHandler& nh, const target_data::TargetData& target_data)
    } // Class Constructor End: TargetCartesian()

    
    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetCartesian::TargetCartesian(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        TargetBase(nh, getParamTargetData(param_xml))
    {
        // This constructor delegates the construction of the TargetCartesian-class to:
        // TargetCartesian(ros::NodeHandler& nh, const target_data::TargetData& target_data)
    } // Class Constructor End: TargetCartesian()


    // Get This
    // -------------------------------
    TargetCartesian* TargetCartesian::getThis()
    {
        // Return a pointer to the current instance of this class
        return this;
    } // Function End: getThis()


    // Print Target-Data
    // -------------------------------
    void TargetCartesian::printTargetData()
    {
        // Call target-base implementation to print header information
        TargetBase::printTargetData();

        // Print information on local target cartesian data to terminal
        ROS_INFO_STREAM("Cartesian Target:");
        ROS_INFO_STREAM("   Ref-Frame: "    << target_data_.cartesian.ref_frame);
        ROS_INFO_STREAM("   Position:");
        ROS_INFO_STREAM("       x: "        << target_data_.cartesian.pose_config.position.x << " [m]");
        ROS_INFO_STREAM("       y: "        << target_data_.cartesian.pose_config.position.y << " [m]");
        ROS_INFO_STREAM("       z: "        << target_data_.cartesian.pose_config.position.z << " [m]");
        ROS_INFO_STREAM("   Orientation:");
        ROS_INFO_STREAM("       rx: "       << target_data_.cartesian.pose_config.orientation.x << " [deg]");
        ROS_INFO_STREAM("       ry: "       << target_data_.cartesian.pose_config.orientation.y << " [deg]");
        ROS_INFO_STREAM("       rz: "       << target_data_.cartesian.pose_config.orientation.z << " [deg]");
        ROS_INFO_STREAM(" ");
    } // Function End: printTargetData()


    // Get Target Parameter Data
    // -------------------------------
    // (Function Overloading)
    target_data::TargetData TargetCartesian::getParamTargetData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Report to terminal:
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__
            << ": Loading Target Parameter [" + param_name + "]";);
        
        // Check parameter server for target parameter
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            std::string error_msg = CLASS_PREFIX + __FUNCTION__
                + ": Failed! Target Parameter [" + param_name + "] is NOT found";

            // Report to terminal and throw run-time exception
            ROS_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }

        // Function return
        return this->getParamTargetData(param_xml);
    } // Function End: getParamTargetData()


    // Get Target Parameter Data
    // -------------------------------
    // (Function Overloading)
    target_data::TargetData TargetCartesian::getParamTargetData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

        // Define local variable(s)
        target_data::TargetData target_data;

        // Call target-base implementation to gather header information
        target_data = TargetBase::getParamTargetData(param_xml);

        // Check if given parameter has "pose"-member
        if(!Toolbox::Parameter::checkMember(param_xml, "pose"))
        {
            // Parameter is configurated incorrectly
            std::string error_msg = CLASS_PREFIX + __FUNCTION__
                + ": Failed! Given Target-Cartesian parameter is configured incorrectly";

            // Report to terminal and throw run-time exception
            ROS_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }

        // Try to load parameter data
        try
        {
            // Read and validate parameter data
            target_data.cartesian = getParamTargetCartesian(param_xml);
        }
        // Catch Exception(s)
        catch (const std::exception& e) 
        {
            // Failed to get parameter
            std::string error_msg = CLASS_PREFIX + __FUNCTION__
                + ": Failed! Parameter(s) related to Target [" + target_data.header.name + "]" 
                + " is either missing or configured incorrectly";

            // Exception details
            std::cerr << e.what() << std::endl;

            // Report to terminal and throw run-time exception
            ROS_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        } 
        // Function return
        return target_data;
    } // Function End: getParamTargetData()


    // Get Target-Cartesian Parameter Data
    // -------------------------------
    target_data::TargetCartesian TargetCartesian::getParamTargetCartesian(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)

        // Define local variable(s)
        target_data::TargetCartesian target_cartesian;

        // Load, validate and assign parameter data
        target_cartesian.name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");
        target_cartesian.ref_frame = Toolbox::Parameter::getParamData<std::string>(param_xml, "ref_frame");
        target_cartesian.pose_config.position.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "x");
        target_cartesian.pose_config.position.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "y");
        target_cartesian.pose_config.position.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "z");
        target_cartesian.pose_config.orientation.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rx");
        target_cartesian.pose_config.orientation.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "ry");
        target_cartesian.pose_config.orientation.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rz");
        target_cartesian.pose = Toolbox::Convert::poseRPYToPose(target_cartesian.pose_config);

        // Function return
        return target_cartesian;
    } // Function End: getParamTargetCartesian()


} // End Namespace: Target