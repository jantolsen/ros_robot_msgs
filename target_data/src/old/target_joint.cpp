// Target Type - Joint
// -------------------------------
// Description:
//      Robot system target joint.
//      Target joint provides behaviour and methods related to a custom target joint type.
//      Collects information of custom target data from the parameter server
//      Structures and sorts the gathered information to respective target-data message-type.
//      Providing functionalities to interact with and broadcast related target-data
// 
//      The target joint is implemented as a strategy class as a part of the strategy-pattern 
//      implementation of robot-target. This strategy implements the interface (target type interface)
//      and provides unique implementation of the behaviour and methods related to target joint type.
//
// Version:
//  0.2 -   Overhaul of Target implementation.
//          Introduding strategy-pattern, utilizing interface, strategies and context
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

} // End Namespace: Target