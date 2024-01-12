// Target Context
// -------------------------------
// Description:
//      Robot system target context
//      Collects information on custom target from the parameter server. 
//      Structures and sorts the information into a target message-type.
//      Provides functionality to interact and broadcast related target information.
//
// Version:
//  0.1 - Initial Version
//        [09.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "target_msgs/target_context.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Context Class
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetContext::CLASS_PREFIX = "TargetContext::";

    // // Class constructor
    // // -------------------------------
    // // (Constructor Overloading)
    // TargetContext::TargetContext(
    //     ros::NodeHandle& nh)
    // :
    //     nh_(nh),
    //     target_data_(target_data),
    //     target_type_map_(initTargetTypeMap())
    // {
    //     // Initialize target-context
    //     // init(target_data);

    //     // Initialize publisher(s)
    //     target_pub_ = nh_.advertise<target_msgs::TargetData>("/target/" + target_data_.name, 1);

    // } // Class Constructor End: TargetHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetContext::TargetContext(
        ros::NodeHandle& nh,
        const target_msgs::TargetData& target_data)
    :
        nh_(nh),
        target_data_(target_data),
        target_type_map_(initTargetTypeMap())
    {
        // Initialize target-context
        // init(target_data);

        // Initialize publisher(s)
        target_pub_ = nh_.advertise<target_msgs::TargetData>("/target/" + target_data_.name, 1);

    } // Class Constructor End: TargetHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetContext::TargetContext(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        TargetContext(nh, loadParamData(param_name).value())
    {
        // Initialize target-context
        // init(param_name);
    } // Class Constructor End: TargetHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetContext::TargetContext(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        TargetContext(nh, loadParamData(param_xml).value())
    {
        // Initialize target-context
        // init(param_xml);
    } // Class Constructor End: TargetHandler()


    // Class Desctructor
    // -------------------------------
    TargetContext::~TargetContext()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~TargetContext()


    // Publish Target Data
    // -------------------------------
    void TargetContext::publishTarget()
    {
        // Publish the target data
        target_pub_.publish(target_data_);
    }  // Function End: publishTarget()


    // Get Target Data
    // -------------------------------
    target_msgs::TargetData TargetContext::getTargetData()
    {
        // Return local target data
        return target_data_;
    }  // Function End: getTargetData()


    // Update Target Data
    // -------------------------------
    void TargetContext::updateTargetData(
        target_msgs::TargetData target_data)
    {
        // Update local target data
        target_data_ = target_data;
    }  // Function End: updateTargetData()


    // Print Target data
    // -------------------------------
    void TargetContext::printTargetData()
    {
        // Print information on local target data to terminal
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("Target:");
        ROS_INFO_STREAM("--------------------");
        ROS_INFO_STREAM("Name: " << target_data_.name);
        ROS_INFO_STREAM("Type: " << target_data_.type_name);

        // Determine target type
        switch (static_cast<TargetType>(target_data_.type))
        {
            // Target type: Joint
            case TargetType::JOINT:
                // Print target joint data
                printTargetJointData();

                // Case break
                break;

            // Target type: Cartesian
            case TargetType::CARTESIAN:
                // Print target cartesian data
                printTargetCartesianData();

                // Case break
                break;
            // Unrecognized target type
            default:
                // Target type is not recognized
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                    << ": Failed! Target type [" << target_data_.type << "] is NOT recognized");
                // Function return
                return;
        } // Switch End: target_data_.type
    } // Function End: printTargetData()
    

    // Initialize Target Context
    // -------------------------------
    // (Function Overloading)
    void TargetContext::init(const target_msgs::TargetData& target_data)
    {
        // Initialize local target-type map
        target_type_map_ = initTargetTypeMap();

        // Initialize local target data
        target_data_ = target_data;

        // Initialize publisher(s)
        target_pub_ = nh_.advertise<target_msgs::TargetData>("/target/" + target_data_.name, 1);
    } // Function End: init()


    // Initialize Target Context
    // -------------------------------
    // (Function Overloading)
    void TargetContext::init(const std::string& param_name)
    {
        // Initialize Target-Type map
        target_type_map_ = initTargetTypeMap();

        // Load target parameter data
        auto result_data = loadParamData(param_name);
        if(!result_data)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initialization failed!");

            // Function return
            return;
        }

        // Calling overloaded function
        return init(result_data.value());
    } // Function End: init()


    // Initialize Target Context
    // -------------------------------
    // (Function Overloading)
    void TargetContext::init(const XmlRpc::XmlRpcValue& param_xml)
    {
        // Initialize Target-Type map
        target_type_map_ = initTargetTypeMap();

        // Load target parameter data
        auto result_data = loadParamData(param_xml);
        if(!result_data)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initialization failed!");

            // Function return
            return;
        }

        // Calling overloaded function
        return init(result_data.value());
    } // Function End: init()


    // Load Target Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_msgs::TargetData> TargetContext::loadParamData(
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
        // Function return: Call overloading function
        return loadParamData(param_xml);
    } // Function End: loadParamData() 


    // Load Target Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<target_msgs::TargetData> TargetContext::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the info-message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        target_msgs::TargetData target_data;

        // Define and initialize target-type map
        std::map<std::string, TargetType> target_type_map;

        // Check if given parameter is a struct-type
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Target parameter is NOT a struct");

            // Function return
            return boost::none;
        }

        // Try to load parameters
        try
        {
            // Load, validate and assign parameter data
            target_data.name = Toolbox::Parameter::loadParamData<std::string>(param_xml, "name");
            target_data.type_name = Toolbox::Parameter::loadParamItemKey<std::string>(target_type_map, param_xml, "type");
            target_data.type = Toolbox::Parameter::loadParamItemValue<int>(target_type_map, param_xml, "type");
            target_data.visible = Toolbox::Parameter::loadParamData<bool>(param_xml, "visible");
        }
        // Catch Exception(s)
        catch (const std::exception& e) 
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to Target [" << target_data.name << "]" 
                << " is either missing or configured incorrectly");

            // Exception details
            // std::cerr << e.what() << std::endl;

            // Function return
            return boost::none;
        } 
        
        // // Determine target type
        // switch (static_cast<TargetType>(target_data.type))
        // {
        //     // Target type: Joint
        //     case TargetType::JOINT:
        //         // Load, validate and assign target-joint parameter data
        //         // boost::optional<target_msgs::TargetJoint> result_joint_data = loadParamJointData(param_xml["position"]);
        //         // if(!result_joint_data) return boost::none;

        //         // // Parameter loading was successful
        //         // target_data.joint = result_joint_data.value();

        //         // Case break
        //         break;

        //     // Target type: Cartesian
        //     case TargetType::CARTESIAN:
        //         // Load, validate and assign target-cartesian parameter data
        //         // boost::optional<target_msgs::TargetCartesian> result_cartesian_data = loadParamCartesianData(param_xml["pose"]);
        //         // if(!result_cartesian_data) return boost::none;

        //         // // Parameter loading was successful
        //         // target_data.cartesian = result_cartesian_data.value();

        //         // Case break
        //         break;

        //     // Unrecognized target type
        //     default:
        //         // Target type is not recognized
        //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //             << ": Failed! Target type [" << target_data.type << "] is NOT recognized");
        //         // Function return
        //         return boost::none;
        // } // Switch End: target_data.type

        // Function return
        return target_data;
    } // Function End: loadParamData()


    // Load Target Joint Data
    // -------------------------------
    boost::optional<target_msgs::TargetJoint> TargetContext::loadParamJointData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Define local variable(s)
        target_msgs::TargetJoint target_joint_data;
        
        // // Evaluate parameter(s) type
        // switch (param_xml.getType())
        // {
        //     // Array: 
        //     // Target-Joint data is configured as an array
        //     case XmlRpc::XmlRpcValue::TypeArray:
        //         // // Resize target joint data to match the number of joints
        //         // target_joint_data.position_deg.resize(param_xml.size());

        //         // // Load, validate and assign parameter data
        //         // if (!Toolbox::Parameter::loadParamData<std:.vector<double>>(target_joint_data.position_deg[i], param_xml, "position")) params_valid =  false;

        //         // // Iterate through all joints   
        //         // for (size_t i = 0; i < param_xml.size(); i++)
        //         // {
        //         //     // Load, validate and assign parameter data
        //         //     if (!Toolbox::Parameter::loadParamData<std:.vector<double>>(target_joint_data.position_deg[i], param_xml, "position")) params_valid =  false;
        //         // }

        //         // Case break
        //         break;

        //     // Struct: 
        //     // Target-Joint data is configured as a struct
        //     case XmlRpc::XmlRpcValue::TypeStruct:
        //         // // Resize target joint data to match the number of joints
        //         // target_joint_data.position_deg.resize(param_xml.size());

        //         // if (!Toolbox::Parameter::loadParamData<double>(user_frame_data.pose_rpy.position.x, param_xml["position"], "q0")) params_valid =  false;

        //         // Case break
        //         break;

        //     // Unrecognized target type
        //     default:
        //         // Target type is not recognized
        //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //             << ": Failed! Target type is NOT recognized");
        //         // Function return
        //         return boost::none;

        // } // Switch End: param_xml.getType()


        // // Check if given parameter is a array-type
        // if(Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        // {
        //     // Parameter is not a array
        //     ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //         << ": Failed! Given Target-Joint parameter is NOT an array");

        //     // Function return
        //     return boost::none;
        // }

        // // Initialize a flag to track the validation of the parameter loading
        // bool params_valid = true;

        // Load, validate and assign parameter data
        // if (!Toolbox::Parameter::loadParamData<std::vector<double>>(target_joint_data.position_deg, param_xml, "position_deg")) params_valid =  false;
        return boost::none;
    } // Function End: loadParamJointData()


    // Load Target Cartesian Data
    // -------------------------------
    boost::optional<target_msgs::TargetCartesian> TargetContext::loadParamCartesianData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        return boost::none;
    } // Function End: loadParamCartesianData()


    // Print Target Joint Data
    // -------------------------------
    void TargetContext::printTargetJointData()
    {
        // Print information of local target joint data to terminal
        
        ROS_INFO_STREAM("Joint Target:");
        ROS_INFO_STREAM("   Position:");
        // Iterate through all joint positions
        for (size_t i = 0; i < target_data_.joint.position_deg.size(); i++)
        {
            // Print joint position for each joint
            ROS_INFO_STREAM("       q[" << i+1 << "]: " << target_data_.joint.position_deg[i] << " [deg]");
        }
        ROS_INFO_STREAM(" ");
    } // Function End: printTargetJointData()


    // Print Target Cartesian Data
    // -------------------------------
    void TargetContext::printTargetCartesianData()
    {
        // Print information of local target cartesian data to terminal
        ROS_INFO_STREAM("Cartesian Target:");
        ROS_INFO_STREAM("   Reference-Frame: "  << target_data_.cartesian.ref_frame);
        ROS_INFO_STREAM("   Position:");
        ROS_INFO_STREAM("       x: "    << target_data_.cartesian.pose_rpy.position.x << " [m]");
        ROS_INFO_STREAM("       y: "    << target_data_.cartesian.pose_rpy.position.y << " [m]");
        ROS_INFO_STREAM("       z: "    << target_data_.cartesian.pose_rpy.position.z << " [m]");
        ROS_INFO_STREAM("   Orientation:");
        ROS_INFO_STREAM("       rx: "   << target_data_.cartesian.pose_rpy.orientation.x << " [deg]");
        ROS_INFO_STREAM("       ry: "   << target_data_.cartesian.pose_rpy.orientation.y << " [deg]");
        ROS_INFO_STREAM("       rz: "   << target_data_.cartesian.pose_rpy.orientation.z << " [deg]");
        ROS_INFO_STREAM(" ");
    } // Function End: printTargetCartesianData()


    // Initialize Target Type Map
    // -------------------------------
    std::map<std::string, TargetType> TargetContext::initTargetTypeMap()
    {
        // Initialize and populate map
        std::map<std::string, TargetType> target_type_map =
        {
            {"JOINT",       TargetType::JOINT},
            {"CARTESIAN",   TargetType::CARTESIAN}
        };

        // Function return
        return target_type_map;
    } // Function End: initTargetTypeMap()

} // End Namespace: Target