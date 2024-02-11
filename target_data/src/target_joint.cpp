// Target Joint
// -------------------------------
// Description:
//      Robot system target joint.
//      Target joint provides behaviour and methods related to a custom target joint type.
//      Collects information of custom target data from the parameter server
//      Structures and sorts the gathered information to respective target-data message-type.
//      Providing functionalities to interact with and broadcast related target-data
//
//      The target joint is implemented as a part of the strategy-pattern implementation of the 
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
    #include "target_data/target_joint.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Joint Class
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetJoint::CLASS_PREFIX = "TargetJoint::";


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetJoint::TargetJoint(
        ros::NodeHandle& nh,
        const target_data::TargetData& target_data)
    :
        TargetBase(nh, target_data)
    {

    } // Class Constructor End: TargetBase()


    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetJoint::TargetJoint(
        ros::NodeHandle& nh,
        const std::string& param_name)
    :
        TargetBase(nh, getParamTargetData(param_name))
    {
        // This constructor delegates the construction of the TargetJoint-class to:
        // TargetJoint(ros::NodeHandler& nh, const target_data::TargetData& target_data)
    } // Class Constructor End: TargetJoint()

    
    // Class constructor
    // -------------------------------
    // (Constructor Overloading)
    TargetJoint::TargetJoint(
        ros::NodeHandle& nh,
        const XmlRpc::XmlRpcValue& param_xml)
    :
        TargetBase(nh, getParamTargetData(param_xml))
    {
        // This constructor delegates the construction of the TargetJoint-class to:
        // TargetJoint(ros::NodeHandler& nh, const target_data::TargetData& target_data)
    } // Class Constructor End: TargetJoint()


    // Get This
    // -------------------------------
    TargetJoint* TargetJoint::getThis()
    {
        // Return a pointer to the current instance of this class
        return this;
    } // Function End: getThis()

} // End Namespace: Target