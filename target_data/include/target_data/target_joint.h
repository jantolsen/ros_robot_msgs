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

// Include guard:
// -------------------------------
#ifndef TARGET_JOINT_H       
#define TARGET_JOINT_H

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

    // Target-Data Messages
    #include "target_data/TargetData.h"
    #include "target_data/TargetJoint.h"


// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Type Joint Class
    // -------------------------------
    /** \brief Robot system target joint
    *
    * Target joint provides behaviour and methods related to a custom target joint type.
    * Collects information of custom target data from the parameter server
    * Structures and sorts the gathered information to respective target-data message-type.
    * Providing functionalities to interact with and broadcast related target-data
    * 
    * The target joint is implemented as a strategy class as a part of the strategy-pattern 
    * implementation of robot-target. This strategy implements the interface (target type interface)
    * and provides unique implementation of the behaviour and methods related to target joint type.
    */
    class TargetJoint : public TargetInterface
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

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

    }; // End Class: TargetJoint
} // End Namespace: Target
#endif // TARGET_JOINT_H 