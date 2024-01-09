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
    } // Class Constructor End: TargetHandler()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetContext::TargetContext(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        // Constructor delegation
        TargetContext(nh, loadParamData(param_name).value())
    {
        // This constructor delegates the construction of the TargetContext-class to:
        // TargetContext(ros::NodeHandler& nh, const target_msgs::TargetData& user_frame_data)
    } // Class Constructor End: TargetHandler()

    
    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetContext::TargetContext(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        // Constructor delegation
        TargetContext(nh, loadParamData(param_xml).value())
    {
        // This constructor delegates the construction of the TargetContext-class to:
        // TargetContext(ros::NodeHandler& nh, const target_msgs::TargetData& user_frame_data)
    } // Class Constructor End: TargetHandler()


    // Class Desctructor
    // -------------------------------
    TargetContext::~TargetContext()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~TargetContext()


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
                <<  ": Failed! Target Parameter [" << param_name << "] not found");

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

        // Check if given parameter is a struct-type
        if(!Toolbox::Parameter::checkDataType(param_xml, XmlRpc::XmlRpcValue::TypeStruct))
        {
            // Parameter is not a struct
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Failed! Given Target parameter is not a struct");

            // Function return
            return boost::none;
        }

        // Initialize a flag to track the validation of the parameter loading
        bool params_valid = true;

        // Function return
        return target_data;
    } // Function End: loadParamData()