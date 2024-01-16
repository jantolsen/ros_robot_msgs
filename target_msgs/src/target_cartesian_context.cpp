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
        TargetContext(nh, loadParamData(param_xml).value())
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


} // End Namespace: Target