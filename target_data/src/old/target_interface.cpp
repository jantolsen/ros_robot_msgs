// Target Type - Interface
// -------------------------------
// Description:
//      Robot system target type interface.
//      Target type interface defines common behaviour and methods related to robot system target.
//      Collects information of custom target data from the parameter server
//      Structures and sorts the gathered information to respective target-data message-type.
//      Provides functionalities to interact with and broadcast related target-data
// 
//      The target type interface is implemented as an absract class as a part of the strategy-pattern 
//      implementation of robot-target. This interface defines the common behaviour and methods
//      for all strategies (target-types). The related strategies must then implement this interface
//      to provide their unique implementation of the behaviour and methods.
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
    #include "target_data/target_interface.h"

// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Interface Class
    // -------------------------------

    // Class Desctructor
    // -------------------------------
    TargetInterface::~TargetInterface()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
            << ": Destructor called");

    } // Class Desctructor End: ~TargetContext()
    

    // Get Target Data
    // -------------------------------
    target_msgs::TargetData TargetInterface::getTargetData()
    {
        // Return local target data
        return target_data_;
    }  // Function End: getTargetData()


    // Update Target Data
    // -------------------------------
    void TargetInterface::updateTargetData(
        target_msgs::TargetData target_data)
    {
        // Update local target data
        target_data_ = target_data;
    }  // Function End: updateTargetData()

} // End Namespace: Target