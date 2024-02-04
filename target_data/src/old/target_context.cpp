// Target - Context
// -------------------------------
// Description:
//      Robot system target-context.
//      Collects information of custom target data from the parameter server
//      Structures and sorts the gathered information to respective target-data message-type.
//      Provides functionalities to interact with and broadcast related target-data
// 
//      The target-context is implemented as strategy delegation class as a part of the strategy-pattern 
//      implementation of robot-target. This context delegates the behaviour and methods to the matching 
//      strategy (target-type). The context maintains reference to the target-type and interacts with 
//      it via the common interface.
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
    #include "target_data/target_context.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Context Class
    // -------------------------------

    // Constants
    // -------------------------------
    const std::string TargetContext::CLASS_PREFIX = "TargetContext::";

} // End Namespace: Target