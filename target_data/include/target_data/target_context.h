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

// Include guard:
// -------------------------------
#ifndef TARGET_CONTEXT_H       
#define TARGET_CONTEXT_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>
    #include <map>

    // Ros
    #include <ros/ros.h>

    // TF2
    #include <tf2_ros/transform_broadcaster.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Target-Data
    #include "target_data/target_interface.h"
    #include "target_data/target_joint.h"
    #include "target_data/target_cartesian.h"
    #include "target_data/target_joint_extaxis.h"

    // Target-Data Messages
    #include "target_data/TargetData.h"
    #include "target_data/TargetJoint.h"
    #include "target_data/TargetCartesian.h"
    #include "target_data/TargetJointExtAxis.h"


// Namespace: Target
// -------------------------------
namespace Target
{
    // Enums
    // -------------------------------
    // Target Type 
    // (Matches the types defined in TargetData.msg)
    enum TargetType
    {
        JOINT,
        CARTESIAN,
        JOINT_EXTAXIS
    };


    // Target Context Class
    // -------------------------------
    /** \brief Robot system target-context
    *
    * Collects information of custom target data from the parameter server
    * Structures and sorts the gathered information to respective target-data message-type.
    * Provides functionalities to interact with and broadcast related target-data
    * 
    * The target-context is implemented as strategy delegation class as a part of the strategy-pattern 
    * implementation of robot-target. This context delegates the behaviour and methods to the matching 
    * strategy (target-type). The context maintains reference to the target-type and interacts with 
    * it via the common interface.
    */
    class TargetContext
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Define Shared-Pointer of Class-Object
            typedef typename std::shared_ptr<TargetCartesian> Ptr;

        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Class Local Member(s)
            // -------------------------------
            TargetInterface* targetInterface_;
            target_data::TargetData target_data_;
            std::map<std::string, TargetType> target_type_map_;
            std::vector<std::string> target_type_names_vec_;
        

    }; // End Class: TargetContext
} // End Namespace: Target
#endif // TARGET_CONTEXT_H 