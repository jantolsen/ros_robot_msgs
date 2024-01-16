// Target Context - Joint
// -------------------------------
// Description:
//      Robot system target-joint context
//      Collects information on custom target-joint from the parameter server. 
//      Structures and sorts the information into a target-joint message-type.
//      Provides functionality to interact and broadcast related target-joint information.
//
// Version:
//  0.2 -   Overhaul of Target-Context implementation.
//          Split target-type into seperate classes.
//          [16.01.2024]  -   Jan T. Olsen
//  0.1 -   Initial Version
//          [09.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "target_msgs/target_joint_context.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Joint Context Class
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetJointContext::CLASS_PREFIX = "TargetJointContext::";


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetJointContext::TargetJointContext(
        ros::NodeHandle& nh,
        const target_msgs::TargetJoint& target_joint)
    :
        nh_(nh),
        target_joint_(target_joint)
    {

    } // Class Constructor End: TargetHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetJointContext::TargetJointContext(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        TargetJointContext(nh, loadParamData(param_name).value())
    {
        // This constructor delegates the construction of the TargetJointContext-class to:
        // TargetJointContext(ros::NodeHandler& nh, const target_msgs::TargetData& target_data)
    } // Class Constructor End: TargetHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetJointContext::TargetJointContext(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        TargetJointContext(nh, loadParamData(param_xml).value())
    {
        // This constructor delegates the construction of the TargetJointContext-class to:
        // TargetJointContext(ros::NodeHandler& nh, const target_msgs::TargetData& target_data)
    } // Class Constructor End: TargetHandler()


    // Class Desctructor
    // -------------------------------
    TargetJointContext::~TargetJointContext()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~TargetJointContext()


    // Get Target-Joint Data
    // -------------------------------
    target_msgs::TargetJoint TargetJointContext::getTargetJoint()
    {
        // Return local target data
        return target_joint_;
    }  // Function End: getTargetJoint()


    // Update Target-Joint Data
    // -------------------------------
    void TargetJointContext::updateTargetJoint(
        target_msgs::TargetJoint target_joint)
    {
        // Update local target data
        target_joint_ = target_joint;
    }  // Function End: updateTargetJoint()


    // Load Target-Joint Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_msgs::TargetJoint> TargetJointContext::loadParamData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for Target parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                <<  ": Failed! Target-Joint Parameter [" << param_name << "] is NOT found");

            // Function return
            return boost::none;
        }
        // Function return
        return loadParamData(param_xml);
    } // Function End: loadParamData()


    // Load Target-Joint Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_msgs::TargetJoint> TargetJointContext::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        std::string target_name;
        target_msgs::TargetJoint target_joint;
        target_msgs::TargetExtAxis ext_axis_data;

        // Check if given parameter is a struct-type
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Target-Joint parameter is NOT a struct");

            // Function return
            return boost::none;
        }

        // Check if given target-joint parameter has "position"-member
        if(!Toolbox::Parameter::checkMember(param_xml, "position"))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Target-Joint parameter is configured incorrectly");

            // Function return
            return boost::none;
        }

        // Load, validate and assign parameter data
        try
        {
            // Get target-joint parameter-data
            target_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");

            // Special-case: target-joint is configured with external-axis
            if(Toolbox::Parameter::getParamData<bool>(param_xml["ext_axis"], "installed", false))
            {
                // Get External-Axis parameter-data
                ext_axis_data = getParamExtAxis(param_xml);

                // Get Target-Joint parameter-data (with External-Axis)
                target_joint = getParamTargetJointExtAxis(param_xml, ext_axis_data);
            }
            // Normal-case: target is NOT configured with external-axis
            else
            {
                // Get Target-Joint parameter-data
                target_joint = getParamTargetJoint(param_xml);
            }
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
        return target_joint;
    } // Function End: loadParamData()


    // Print Target-Joint Data
    // -------------------------------
    // (Function Overloading)
    void TargetJointContext::printTargetJoint()
    {
        // Special-case: target-joint is configured with external-axis
        if (target_joint_.external_axis.installed)
        {
            // Call print function for target-joint with external-axis
            printTargetJointExtAxis();
            return;
        }

        // Normal-case: target-joint is NOT configured with external-axis
        // Print information on local target-joint data to terminal
        ROS_INFO_STREAM("Joint Target:");
        ROS_INFO_STREAM("   Number of Joints: "     << target_joint_.num_joints);
        ROS_INFO_STREAM("   External Axis: "        << (target_joint_.external_axis.installed ? "TRUE" : "FALSE"));
        ROS_INFO_STREAM("   Position:");
        // Iterate through all joint positions
        for (size_t i = 0; i < target_joint_.num_joints; i++)
        {
            // Special-case: firts joint (external axis)
            if(i == 0)
            {
                // Continue to next iteration
                continue;
            }
            // Normal-case: all other joints
            else
            {
                // Print joint position for respective joint
                ROS_INFO_STREAM("       q[" << i << "]: " << target_joint_.position_deg[i] << " [deg]");
            }
        }
        ROS_INFO_STREAM(" ");
    } // Function End: printTargetJoint()


    // Print Target-Joint Data 
    // (with External Axis)
    // -------------------------------
    // (Function Overloading)
    void TargetJointContext::printTargetJointExtAxis()
    {
        // Print information on local target-joint data to terminal
        ROS_INFO_STREAM("Joint Target:");
        ROS_INFO_STREAM("   Number of Joints: "     << target_joint_.num_joints);
        ROS_INFO_STREAM("   External Axis: "        << (target_joint_.external_axis.installed ? "TRUE" : "FALSE"));
        ROS_INFO_STREAM("   External Axis Type: "   << target_joint_.external_axis.type_name);
        ROS_INFO_STREAM("   Position:");
        // Iterate through all joint positions
        for (size_t i = 0; i < target_joint_.num_joints; i++)
        {
            // Special-case for firts joint (external axis)
            if(i == 0)
            {
                // External axis unit
                std::string ext_axis_unit = (target_joint_.external_axis.type == static_cast<int>(JointAxisType::ROTATION)) ? " [deg]" : " [m]";
                // Print joint position for external axis
                ROS_INFO_STREAM("       q[" << i << "]: " << target_joint_.position_deg[i] << ext_axis_unit);
            }
            // Normal-case: all other joints
            else
            {
                // Print joint position for respective joint
                ROS_INFO_STREAM("       q[" << i << "]: " << target_joint_.position_deg[i] << " [deg]");
            }
        }
        ROS_INFO_STREAM(" ");
    } // Function End: printTargetJointExtAxis()


    // Get Target-Joint Parameter Data
    // -------------------------------
    // (Function Overloading)
    target_msgs::TargetJoint TargetJointContext::getParamTargetJoint(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Define local variable(s)
        target_msgs::TargetJoint target_joint;
        std::string target_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");

        // Determine and check number of joints by evaluating joint-position parameter-size
        int num_joints = param_xml["position"].size();
        if(num_joints == 7)
        {
            // Invalid parameter configuration
            std::string error_msgs = CLASS_PREFIX + __FUNCTION__ 
                + ": Failed! Parameter(s) related to Target-Joint [" + target_name + "]" 
                + " with number of joint-positions [" + std::to_string(num_joints) + "]. "
                + " Indicates target-joint with external-axis, but no external-axis parameters is found";

            // Report to terminal and throw runtime exception
            ROS_ERROR_STREAM(error_msgs);
            throw std::runtime_error("Runtime exception! " + error_msgs);
        }
        else if (num_joints != 6)
        {
            // Invalid parameter configuration
            std::string error_msgs = CLASS_PREFIX + __FUNCTION__ 
                + ": Failed! Parameter(s) related to Target-Joint [" + target_name + "]" 
                + " is configured with an invalid number of joint-positions [" + std::to_string(num_joints) + "]";

            // Report to terminal and throw runtime exception
            ROS_ERROR_STREAM(error_msgs);
            throw std::runtime_error("Runtime exception! " + error_msgs);
        }

        // Determine configuration structure by evaluating joint-position parameter-type
        switch (param_xml["position"].getType())
        {
            // Array: 
            // Parameter of Target-Joint is configured as an array
            case XmlRpc::XmlRpcValue::TypeArray:
                // Load, validate and assign parameter data
                target_joint.name = target_name;
                target_joint.num_joints = num_joints;
                target_joint.position_deg = Toolbox::Parameter::getParamData<std::vector<double>>(param_xml, "position");
                target_joint.position = Toolbox::Convert::degToRad(target_joint.position_deg);

            // Struct: 
            // Parameter of Target-Joint is configured as a struct
            case XmlRpc::XmlRpcValue::TypeStruct:
                // Load, validate and assign parameter data
                target_joint.name = target_name;
                target_joint.num_joints = num_joints;
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q1"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q2"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q3"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q4"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q5"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q6"));
                target_joint.position = Toolbox::Convert::degToRad(target_joint.position_deg);
            
            // Unknown: 
            // Parameter of Parameter of Target-Joint is configured with unknown type
            default:
                // Failed to get parameter(s)
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter(s) related to Target-Joint [" << target_name <<"] is wrongly defined." 
                    << " Target-Joint parameters are neither an array nor struct");
                break;
        } // Switch End: param_xml["position"].getType()

        // Function return
        return target_joint;
    } // Function End: getParamTargetJoint()

    
    // Get Target-Joint Parameter Data 
    // (with External Axis)
    // -------------------------------
    // (Function Overloading)
    target_msgs::TargetJoint TargetJointContext::getParamTargetJointExtAxis(
        const XmlRpc::XmlRpcValue& param_xml,
        const target_msgs::TargetExtAxis& ext_axis_data)
    {
        // Define local variable(s)
        target_msgs::TargetJoint target_joint;
        std::string target_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");

        // Determine and check number of joints by evaluating joint-position parameter-size
        int num_joints = param_xml["position"].size();
        if(num_joints != 7)
        {
            // Invalid parameter configuration
            std::string error_msgs = CLASS_PREFIX + __FUNCTION__ 
                + ": Failed! Parameter(s) related to Target-Joint [" + target_name + "]" 
                + " is given with external-axis configuration, "
                + " but has an invalid number of joint-positions [" + std::to_string(num_joints) + "]";

            // Report to terminal and throw runtime exception
            ROS_ERROR_STREAM(error_msgs);
            throw std::runtime_error("Runtime exception! " + error_msgs);
        }

        // Determine configuration structure by evaluating joint-position parameter-type
        switch (param_xml["position"].getType())
        {
            // Array: 
            // Parameter of Target-Joint is configured as an array
            case XmlRpc::XmlRpcValue::TypeArray:
                // Load, validate and assign parameter data
                target_joint.name = target_name;
                target_joint.num_joints = num_joints;
                target_joint.position_deg = Toolbox::Parameter::getParamData<std::vector<double>>(param_xml, "position");
                target_joint.position = Toolbox::Convert::degToRad(target_joint.position_deg);

                // Check External-Axis type for correct conversion
                if(ext_axis_data.type == static_cast<int>(JointAxisType::LINEAR))
                {
                    // Keep original parameter-joint value
                    target_joint.position[0] = target_joint.position_deg[0];
                }
                break;

            // Struct: 
            // Parameter of Target-Joint is configured as a struct
            case XmlRpc::XmlRpcValue::TypeStruct:
                // Load, validate and assign parameter data
                target_joint.name = target_name;
                target_joint.num_joints = num_joints;
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q0"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q1"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q2"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q3"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q4"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q5"));
                target_joint.position_deg.push_back(Toolbox::Parameter::getParamData<double>(param_xml["position"], "q6"));
                target_joint.position = Toolbox::Convert::degToRad(target_joint.position_deg);

                // Check External-Axis type for correct conversion
                if(ext_axis_data.type == static_cast<int>(JointAxisType::LINEAR))
                {
                    // Keep original parameter-joint value
                    target_joint.position[0] = target_joint.position_deg[0];
                }
                break;
            // Unknown: 
            // Parameter of Parameter of Target-Joint is configured with unknown type
            default:
                // Failed to get parameter(s)
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Parameter(s) related to Target-Joint [" << target_name <<"] is wrongly defined." 
                    << " Target-Joint parameters are neither an array nor struct");
                break;
        } // Switch End: param_xml["position"].getType()
        
        // Function return
        return target_joint;
    } // Function End: getParamTargetJointExtAxis()


    // Get External Axis Parameter Data
    // -------------------------------
    target_msgs::TargetExtAxis TargetJointContext::getParamExtAxis(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Define local variable(s)
        target_msgs::TargetExtAxis ext_axis;
        std::map<std::string, JointAxisType> joint_axis_type_map = initJointAxisTypeMap();
        std::vector<std::string> joint_axis_type_names =  initJointAxisTypeNames(joint_axis_type_map);

        // Load, validate and assign parameter data
        ext_axis.installed = Toolbox::Parameter::getParamData<bool>(param_xml["ext_axis"], "installed");
        ext_axis.type_name = Toolbox::Parameter::getParamData<std::string>(param_xml["ext_axis"], "type", joint_axis_type_names);
        ext_axis.type = Toolbox::Parameter::getParamData<int>(param_xml["ext_axis"], "type", joint_axis_type_map);

        // Function return
        return ext_axis;
    } // Function End: getParamExtAxis()


    // Initialize Joint Axis Type Map
    // -------------------------------
    std::map<std::string, JointAxisType> TargetJointContext::initJointAxisTypeMap()
    {
        // Initialize and populate map
        std::map<std::string, JointAxisType> joint_type_map =
        {
            {"ROTATION",    JointAxisType::ROTATION},
            {"LINEAR",      JointAxisType::LINEAR}
        };

        // Function return
        return joint_type_map;
    } // Function End: initJointAxisTypeMap()


    // Initialize Joint Axis Type Names
    // -------------------------------
    std::vector<std::string> TargetJointContext::initJointAxisTypeNames(
        std::map<std::string, JointAxisType> joint_type_map)
    {
        // Initialize vector
        std::vector<std::string> joint_type_names_vec;

        // Iterate through joint-axis-type map
        for (auto& joint_type : joint_type_map)
        {
            // Append joint-axis-type name found within joint-axis-type map
            joint_type_names_vec.push_back(joint_type.first);
        }
        // Function return
        return joint_type_names_vec;    
    } // Function End: initJointAxisTypeNames()
} // End Namespace: Target