// Target Context - Cartesian
// -------------------------------
// Description:
//      Robot system target-cartesian context
//      Collects information on custom target-cartesian from the parameter server. 
//      Structures and sorts the information into a target-cartesian message-type.
//      Provides functionality to interact and broadcast related target-cartesian information.
//
// Version:
//  0.2 -   Overhaul of Target-Context implementation.
//          Split target-type into seperate classes.
//          [16.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [16.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "target_msgs/target_cartesian_context.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Cartesian Context Class
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetCartesianContext::CLASS_PREFIX = "TargetCartesianContext::";

    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetCartesianContext::TargetCartesianContext(
        ros::NodeHandle& nh,
        const target_msgs::TargetCartesian& target_cartesian)
    :
        nh_(nh),
        target_cartesian_(target_cartesian)
    {

    } // Class Constructor End: TargetCartesianContext()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetCartesianContext::TargetCartesianContext(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        TargetCartesianContext(nh, loadParamData(param_name).value())
    {
        // This constructor delegates the construction of the TargetCartesianContext-class to:
        // TargetCartesianContext(ros::NodeHandler& nh, const target_msgs::TargetData& target_data)
    } // Class Constructor End: TargetCartesianContext()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetCartesianContext::TargetCartesianContext(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        TargetCartesianContext(nh, loadParamData(param_xml).value())
    {
        // This constructor delegates the construction of the TargetCartesianContext-class to:
        // TargetCartesianContext(ros::NodeHandler& nh, const target_msgs::TargetData& target_data)
    } // Class Constructor End: TargetHandler()


    // Class Desctructor
    // -------------------------------
    TargetCartesianContext::~TargetCartesianContext()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~TargetCartesianContext()


    // Get Target-Cartesian Data
    // -------------------------------
    target_msgs::TargetCartesian TargetCartesianContext::getTargetCartesian()
    {
        // Return local target data
        return target_cartesian_;
    }  // Function End: getTargetJoint()


    // Update Target-Cartesian Data
    // -------------------------------
    void TargetCartesianContext::updateTargetCartesian(
        target_msgs::TargetCartesian target_cartesian)
    {
        // Update local target data
        target_cartesian_ = target_cartesian;
    }  // Function End: updateTargetJoint()


    // Load Target-Cartesian Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_msgs::TargetCartesian> TargetCartesianContext::loadParamData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for Target parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                <<  ": Failed! Target-Cartesian Parameter [" << param_name << "] is NOT found");

            // Function return
            return boost::none;
        }
        // Function return
        return loadParamData(param_xml);
    } // Function End: loadParamData()


    // Load Target-Cartesian Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_msgs::TargetCartesian> TargetCartesianContext::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        std::string target_name;
        target_msgs::TargetCartesian target_cartesian;
        
        // Check if given parameter is a struct-type
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Target-Cartesian parameter is NOT a struct");

            // Function return
            return boost::none;
        }

        // Check if given target-joint parameter has "pose"-member
        if(!Toolbox::Parameter::checkMember(param_xml, "pose"))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Target-Cartesian parameter is configured incorrectly");

            // Function return
            return boost::none;
        }

        // Load, validate and assign parameter data
        try
        {
            // Get target-cartesian parameter-data
            target_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");
            target_cartesian = getParamTargetCartesian(param_xml);    
        }
        // Catch Exception(s)
        catch(const std::exception& e)
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to Target-Joint [" << target_name << "]" 
                << " is either missing or configured incorrectly");

            // Exception details
            std::cerr << e.what() << std::endl;

            // Function return
            return boost::none;
        }

        // Function return
        return target_cartesian;
    } // Function End: loadParamData()


    // Get Target-Cartesian Parameter Data
    // -------------------------------
    target_msgs::TargetCartesian TargetCartesianContext::getParamTargetCartesian(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Define local variable(s)
        target_msgs::TargetCartesian target_cartesian;
        std::string target_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");
        std::vector<double> tmp_target_position;
        std::vector<double> tmp_target_orientation;

        // Determine configuration structure by evaluating cartesian-position parameter-type
        // (assuming orientation is using the same configuration)
        switch (param_xml["pose"]["position"].getType())
        {
            // Array: 
            // Parameter of Target-Cartesian is configured as an array
            case XmlRpc::XmlRpcValue::TypeArray:
                // Load, validate and assign parameter data
                target_cartesian.name = target_name;
                target_cartesian.ref_frame = Toolbox::Parameter::getParamData<std::string>(param_xml, "ref_frame");;
                tmp_target_position = Toolbox::Parameter::getParamData<std::vector<double>>(param_xml["pose"], "position");
                tmp_target_orientation = Toolbox::Parameter::getParamData<std::vector<double>>(param_xml["pose"], "orientation");

                // Check vector size
                if((tmp_target_position.size() != 3) || (tmp_target_orientation.size() != 3))
                {
                    // Invalid parameter configuration
                    std::string error_msgs = CLASS_PREFIX + __FUNCTION__ 
                        + ": Failed! Parameter(s) related to Target-Cartesian [" + target_name + "]" 
                        + " is configured incorrectly. Size of position [" + std::to_string(tmp_target_position.size()) + "]"
                        + " and/or orientation [" + std::to_string(tmp_target_orientation.size()) + "] is wrong!";

                    // Report to terminal and throw runtime exception
                    ROS_ERROR_STREAM(error_msgs);
                    throw std::runtime_error("Runtime exception! " + error_msgs);
                }

                // Assign acquired parameter-data
                target_cartesian.pose_rpy.position.x = tmp_target_position[0];
                target_cartesian.pose_rpy.position.y = tmp_target_position[1];
                target_cartesian.pose_rpy.position.z = tmp_target_position[2];
                target_cartesian.pose_rpy.orientation.x = tmp_target_orientation[0];
                target_cartesian.pose_rpy.orientation.y = tmp_target_orientation[1];
                target_cartesian.pose_rpy.orientation.z = tmp_target_orientation[2];
                target_cartesian.pose = Toolbox::Convert::poseRPYToPose(target_cartesian.pose_rpy);
                break;

            // Struct: 
            // Parameter of Target-Cartesian is configured as a struct
            case XmlRpc::XmlRpcValue::TypeStruct:
                // Load, validate and assign parameter data
                target_cartesian.name = target_name;
                target_cartesian.ref_frame = Toolbox::Parameter::getParamData<std::string>(param_xml, "ref_frame");
                target_cartesian.pose_rpy.position.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "x");
                target_cartesian.pose_rpy.position.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "y");
                target_cartesian.pose_rpy.position.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "z");
                target_cartesian.pose_rpy.orientation.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rx");
                target_cartesian.pose_rpy.orientation.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "ry");
                target_cartesian.pose_rpy.orientation.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rz");
                target_cartesian.pose = Toolbox::Convert::poseRPYToPose(target_cartesian.pose_rpy);
            
            // Unknown: 
            // Parameter of Parameter of Target-Joint is configured with unknown type
            default:
                // Failed to get parameter(s)
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter(s) related to Target-Cartesian [" << target_name <<"] is wrongly defined." 
                    << " Target-Cartesian parameters are neither an array nor struct");
                break;
        } // Switch End: param_cartesian.getType()

        // Function return
        return target_cartesian;
    } // Function End: getParamTargetCartesian()


} // End Namespace: Target