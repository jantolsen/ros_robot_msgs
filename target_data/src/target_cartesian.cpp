// Target Type - Cartesian
// -------------------------------
// Description:
//      Robot system target cartesian.
//      Target cartesian provides behaviour and methods related to a custom target cartesian type.
//      Collects information of custom target data from the parameter server
//      Structures and sorts the gathered information to respective target-data message-type.
//      Providing functionalities to interact with and broadcast related target-data
// 
//      The target cartesian is implemented as a strategy class as a part of the strategy-pattern 
//      implementation of robot-target. This strategy implements the interface (target type interface)
//      and provides unique implementation of the behaviour and methods related to target cartesian type.
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

} // End Namespace: Target