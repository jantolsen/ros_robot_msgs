// User Frame Context
// -------------------------------
// Description:
//      Robot system user frame context
//      Collects information on custom defined user frame from the parameter server. 
//      Structures and sorts the information into a user-frame message-type.
//      Provides functionality related to user-frame interaction.
//
// Version:
//  0.1 - Initial Version
//        [05.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "frame_msgs/user_frame_context.h"

// User-Frame-Handler Class
// -------------------------------

    // Constants
    // -------------------------------
    const std::string UserFrameContext::CLASS_PREFIX = "UserFrameContext::";

    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameContext::UserFrameContext(
        ros::NodeHandle& nh,
        const frame_msgs::UserFrame& user_frame_data)
    :
        nh_(nh),
        user_frame_data_(user_frame_data)
    {
        // Initialize publisher(s)
        user_frame_pub_ = nh_.advertise<frame_msgs::UserFrame>("/user_frame/" + user_frame_data_.name, 1);
    } // Class Constructor End: UserFrameHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameContext::UserFrameContext(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        // Constructor delegation
        UserFrameContext(nh, loadParamData(param_name).value())
    {
        // This constructor delegates the construction of the UserFrameContext-class to:
        // UserFrameContext(ros::NodeHandler& nh, const frame_msgs::UserFrame& user_frame_data)
    } // Class Constructor End: UserFrameHandler()

    
    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    UserFrameContext::UserFrameContext(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        // Constructor delegation
        UserFrameContext(nh, loadParamData(param_xml).value())
    {
        // This constructor delegates the construction of the UserFrameContext-class to:
        // UserFrameContext(ros::NodeHandler& nh, const frame_msgs::UserFrame& user_frame_data)
    } // Class Constructor End: UserFrameHandler()


    // Class Desctructor
    // -------------------------------
    UserFrameContext::~UserFrameContext()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~UserFrameContext()
    

    // Publish User-Frame
    // -------------------------------
    void UserFrameContext::publishUserFrame()
    {
        // Publish the user-frame
        user_frame_pub_.publish(user_frame_data_);
    }  // Function End: publishUserFrame() 


    // Broadcast User-Frame
    // -------------------------------
    void UserFrameContext::broadcastUserFrame()
    {
        // Update Transformation time-stamp for the user-frame
        user_frame_data_.transform_stamped.header.stamp = ros::Time::now();

        // Broadcast the user-frame transformation
        tf2_broadcaster_.sendTransform(user_frame_data_.transform_stamped);
    }  // Function End: broadcastUserFrame() 


    // Publish and Broadcast User-Frame
    // -------------------------------
    void UserFrameContext::publishAndBroadcastUserFrame()
    {
        // Publish the local user-frame
        publishUserFrame();
        // Broadcast the local user-frame
        broadcastUserFrame();
    }  // Function End: publishAndBroadcastUserFrame()

    
    // Get User-Frame Message
    // -------------------------------
    frame_msgs::UserFrame UserFrameContext::getUserFrameData()
    {
        // Return local User-Frame
        return user_frame_data_;
    }  // Function End: getUserFrame()


    // Update User-Frame
    // -------------------------------
    void UserFrameContext::updateUserFrameData(
        frame_msgs::UserFrame user_frame_data)
    {
        // Update local User-Frame
        user_frame_data_ = user_frame_data;
    }  // Function End: updateUserFrame()


    // Print User-Frame
    // -------------------------------
    void UserFrameContext::printUserFrame()
    {
        // Print information of local user-frame to terminal
        ROS_INFO_STREAM(" ");
        ROS_INFO_STREAM("User-Frame:");
        ROS_INFO_STREAM("--------------------");
        ROS_INFO_STREAM("Name: "        << user_frame_data_.name);
        ROS_INFO_STREAM("Ref-Frame: "   << user_frame_data_.ref_frame);
        ROS_INFO_STREAM("   Position:");
        ROS_INFO_STREAM("       x: "    << user_frame_data_.pose_rpy.position.x << " [m]");
        ROS_INFO_STREAM("       y: "    << user_frame_data_.pose_rpy.position.y << " [m]");
        ROS_INFO_STREAM("       z: "    << user_frame_data_.pose_rpy.position.z << " [m]");
        ROS_INFO_STREAM("   Orientation:");
        ROS_INFO_STREAM("       rx: "   << user_frame_data_.pose_rpy.orientation.x << " [deg]");
        ROS_INFO_STREAM("       ry: "   << user_frame_data_.pose_rpy.orientation.y << " [deg]");
        ROS_INFO_STREAM("       rz: "   << user_frame_data_.pose_rpy.orientation.z << " [deg]");
        ROS_INFO_STREAM(" ");
    } // Function End: printUserFrame()
    

    // Load User-Frame Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<frame_msgs::UserFrame> UserFrameContext::loadParamData(
        const std::string& param_name)
    {
        // Define local variable(s)
        XmlRpc::XmlRpcValue param_xml;
        
        // Check parameter server for User-Frame parameters
        if(!ros::param::get(param_name, param_xml))
        {
            // Failed to get parameter
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                <<  ": Failed! User-Frame Parameter [" << param_name << "] not found");

            // Function return
            return boost::none;
        }
        // Function return: Call overloading function
        return loadParamData(param_xml);
    } // Function End: loadParamData() 


    // Load User-Frame Parameter Data
    // -------------------------------
    // (Function Overloading)
    boost::optional<frame_msgs::UserFrame> UserFrameContext::loadParamData(
        const XmlRpc::XmlRpcValue& param_xml)
    {
        // Reads and loads parameter data obtained from the parameter-server
        // Parameters are acquired as XmlRpcValue data-type which acts as generic collector.
        // Elements of the loaded parameters are validated and assigned to the respective element in the info-message-type
        // (data entries of XmlRpcValue needs to be cast to appropriate data-type)
        
        // Define local variable(s)
        frame_msgs::UserFrame user_frame_data;

        // Check if given parameter is a struct-type
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given User-Frame parameter is not a struct");

            // Function return
            return boost::none;
        }

        // Initialize a flag to track the validation of the parameter loading
        bool params_valid = true;

        // Load, validate and assign parameter data
        if (!Toolbox::Parameter::loadParamData<std::string>(user_frame_data.name, param_xml, "name")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<std::string>(user_frame_data.ref_frame, param_xml, "ref_frame")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame_data.pose_rpy.position.x, param_xml["pose"]["position"], "x")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame_data.pose_rpy.position.y, param_xml["pose"]["position"], "y")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame_data.pose_rpy.position.z, param_xml["pose"]["position"], "z")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame_data.pose_rpy.orientation.x, param_xml["pose"]["orientation"], "rx")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame_data.pose_rpy.orientation.y, param_xml["pose"]["orientation"], "ry")) params_valid =  false;
        if (!Toolbox::Parameter::loadParamData<double>(user_frame_data.pose_rpy.orientation.z, param_xml["pose"]["orientation"], "rz")) params_valid =  false;

        // Check if parameter loading was successful
        // (If any parameter failed to load, the flag will be false. Otherwise, it will be true)
        if(!params_valid)
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Parameter(s) related to User-Frame [" << user_frame_data.name << "]  is either missing or configured incorrectly");

            // Function return
            return boost::none;
        }

        // Validate Reference Frame
        if(!validateFrame(user_frame_data.ref_frame))
        {
            // Parameter loading failed
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Reference-Frame [" << user_frame_data.ref_frame << "]" 
                << " related to User-Frame [" << user_frame_data.name << "]  is invalid");

            // Function return
            return boost::none;
        } 

        // Assign Transform data of User-Frame
        geometry_msgs::Pose pose = Toolbox::Convert::poseRPYToPose(user_frame_data.pose_rpy);
        user_frame_data.transform_stamped = Toolbox::Convert::poseToTransform(pose, user_frame_data.ref_frame, user_frame_data.name);

        // Function return
        return user_frame_data;
    } // Function End: loadParamData()


    // Validate Frame
    // -------------------------------
    bool UserFrameContext::validateFrame(
        const std::string& frame_name)
    {
        // Create a TF2 buffer
        tf2_ros::Buffer tf_buffer;

        // Create a TF2 listener
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Check if transformation is available
        try
        {
            // Query the TF2 buffer for a transformation
            tf_buffer.canTransform(frame_name, "world", ros::Time(0), ros::Duration(0.5));

            // Fucntion return
            return true;
        }
        // Catch exception(s)
        catch(tf2::TransformException& ex)
        {
            // Report to terminal
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Tranformation failed!: " << ex.what());

            // Function return
            return false;
        }
    } // Function End: validateFrame()
