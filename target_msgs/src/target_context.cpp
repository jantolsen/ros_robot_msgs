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


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetContext::TargetContext(
        ros::NodeHandle& nh,
        const target_msgs::TargetData& target_data)
    :
        nh_(nh),
        target_data_(target_data)
    {
        // Initialize publisher(s)
        target_pub_ = nh_.advertise<target_msgs::TargetData>("/target/" + target_data_.name, 1);

        // Initialize target-context
        target_type_map_ = initTargetTypeMap();
        target_type_names_vec_ = initTargetTypeNames(target_type_map_);

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
        // This constructor delegates the construction of the TargetContext-class to:
        // TargetContext(ros::NodeHandler& nh, const target_msgs::TargetData& target_data)
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
        // This constructor delegates the construction of the UserFrameContext-class to:
        // TargetContext(ros::NodeHandler& nh, const target_msgs::TargetData& target_data)
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
        // Function return
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
        // Elements of the loaded parameters are validated and assigned to the respective element in the message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        target_msgs::TargetData target_data;
        std::map<std::string, TargetType> target_type_map = initTargetTypeMap();
        std::vector<std::string> target_type_names =  initTargetTypeNames(target_type_map);

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
            // General Parameters
            target_data.name = Toolbox::Parameter::getParamData<std::string>(param_xml, "name");
            target_data.type_name = Toolbox::Parameter::getParamData<std::string>(param_xml, "type", target_type_names);
            target_data.type = Toolbox::Parameter::getParamData<int>(param_xml, "type", target_type_map);
            target_data.visible = Toolbox::Parameter::getParamData<bool>(param_xml, "visible", true);

            // Determine target-type
            switch (target_data.type)
            {
                // Target-Type: Joint
                case static_cast<int>(TargetType::JOINT):
                    // Target-Joint data
                    // Load, validate and assign parameter data
                    target_data.joint.ext_axis = Toolbox::Parameter::getParamData<bool>(param_xml, "ext_axis", false);
                    target_data.joint.position_deg = Toolbox::Parameter::getParamData<std::vector<double>>(param_xml, "position");
                    target_data.joint.position = Toolbox::Convert::degToRad(target_data.joint.position_deg);

                    // Call loading of Target-Joint parameters
                    // target_data.joint = loadParamTargetJoint(param_xml)
                    break;

                // Target-Type: Cartesian
                case static_cast<int>(TargetType::CARTESIAN):
                    // Target-Cartesian data
                    target_data.cartesian.ref_frame = Toolbox::Parameter::getParamData<std::string>(param_xml, "ref_frame");
                    target_data.cartesian.pose_rpy.position.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "x");
                    target_data.cartesian.pose_rpy.position.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "y");
                    target_data.cartesian.pose_rpy.position.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "z");
                    target_data.cartesian.pose_rpy.orientation.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rx");
                    target_data.cartesian.pose_rpy.orientation.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "ry");
                    target_data.cartesian.pose_rpy.orientation.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rz");
                    target_data.cartesian.pose = Toolbox::Convert::poseRPYToPose(target_data.cartesian.pose_rpy);

                    // Call loading of Target-Cartesian parameters
                    // target_data.cartesian = loadParamTargetCartesian(param_xml)
                    break;

                // Target type: Unknown/Invalid 
                default:
                    // Parameter is not a struct
                    ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                        << ": Failed! Target parameter target-type [" << target_data.type << "] is an invalid type");
                    // Function return
                    return boost::none;
            } // End Switch-Case
        }
        // Catch Exception(s)
        catch (const std::exception& e) 
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to Target [" << target_data.name << "]" 
                << " is either missing or configured incorrectly");

            // Exception details
            std::cerr << e.what() << std::endl;

            // Function return
            return boost::none;
        } 

        // Function return
        return target_data;
    } // Function End: loadParamData()

    
    // Load Target-Joint Parameter Data
    // -------------------------------
    target_msgs::TargetJoint TargetContext::loadParamTargetJoint(
        const XmlRpc::XmlRpcValue& param_xml)
    {   
        // Define local variable(s)
        target_msgs::TargetJoint target_joint;
        int num_axis = 6;

        // Load, validate and assign parameter data
        try
        {
            // Target-Joint data
            target_joint.ext_axis = Toolbox::Parameter::getParamData<bool>(param_xml, "ext_axis");
            target_joint.position_deg = Toolbox::Parameter::getParamData<std::vector<double>>(param_xml, "position");
            target_joint.position = Toolbox::Convert::degToRad(target_joint.position_deg);

            // for (int i = 0; i < param_xml["posiiton"].size(); ++i)
            // {
            //     target_joint_data.position
            // }
        }

        // Catch Exception(s)
        catch (const std::exception& e) 
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to Target-Joint" 
                << " is either missing or configured incorrectly");

            // Exception details
            std::cerr << e.what() << std::endl;
        } 

        // Function return
        return target_joint;


        // // Determine if parameter configuration structure
        // // Evaluate parameter type
        // switch (param_xml["position"].getType())
        // {
        //     // Array: 
        //     // Parameter of Target-Joint is configured as an array
        //     case XmlRpc::XmlRpcValue::TypeArray:

        //         break;

        //     // Struct: 
        //     // Parameter of Target-Joint is configured as a struct
        //     case XmlRpc::XmlRpcValue::TypeStruct:
        //         /* code */
        //         break;
            
        //     // Unknown: 
        //     // Parameter of Parameter of Target-Joint is configured with unknown type
        //     default:
        //         // Failed to get parameter(s)
        //         ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
        //             << ": Failed! Target-Joint Parameter(s) [" << param_name <<"] is wrongly defined." 
        //             << " Target-Joint parameters are neither an array nor struct");
        //     default:
        //         break;
        // }

    } // Function End: loadParamTargetJoint


    // Load Target-Joint Parameter Data
    // -------------------------------
    target_msgs::TargetCartesian TargetContext::loadParamTargetCartesian(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Define local variable(s)
        target_msgs::TargetCartesian target_cartesian;

        // Load, validate and assign parameter data
        try
        {
            // Target-Cartesian data
            target_cartesian.ref_frame = Toolbox::Parameter::getParamData<std::string>(param_xml, "ref_frame");
            target_cartesian.pose_rpy.position.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "x");
            target_cartesian.pose_rpy.position.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "y");
            target_cartesian.pose_rpy.position.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["position"], "z");
            target_cartesian.pose_rpy.orientation.x = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rx");
            target_cartesian.pose_rpy.orientation.y = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "ry");
            target_cartesian.pose_rpy.orientation.z = Toolbox::Parameter::getParamData<double>(param_xml["pose"]["orientation"], "rz");
            target_cartesian.pose = Toolbox::Convert::poseRPYToPose(target_cartesian.pose_rpy);
        }

        // Catch Exception(s)
        catch (const std::exception& e) 
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to Target-Cartesian" 
                << " is either missing or configured incorrectly");

            // Exception details
            std::cerr << e.what() << std::endl;
        } 

        // Function return
        return target_cartesian;
    } // Function End: loadParamTargetCartesian


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


    // Initialize Target Type Names
    // -------------------------------
    std::vector<std::string> TargetContext::initTargetTypeNames(
        std::map<std::string, TargetType> target_type_map)
    {
        // Initialize vector
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