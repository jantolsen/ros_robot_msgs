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

// Include guard:
// -------------------------------
#ifndef TARGET_INTERFACE_H       
#define TARGET_INTERFACE_H

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

    // Target Messages
    #include "target_data/TargetData.h"
    #include "target_data/TargetJoint.h"
    #include "target_data/TargetCartesian.h"
    #include "target_data/TargetJointExtAxis.h"


// Namespace: Target
// -------------------------------
namespace Target
{
    // Target Type Interface Class
    // -------------------------------
    /** \brief Robot system target type interface
    *
    * Target type interface defines common behaviour and methods related to robot system target.
    * Collects information of custom target data from the parameter server
    * Structures and sorts the gathered information to respective target-data message-type.
    * Provides functionalities to interact with and broadcast related target-data
    *
    * The target type interface is implemented as an absract class as a part of the strategy-pattern 
    * implementation of robot-target. This interface defines the common behaviour and methods
    * for all strategies (target-types). The related strategies must then implement this interface
    * to provide their unique implementation of the behaviour and methods.
    */
    class TargetInterface
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Class destructor
            // -------------------------------
            /** \brief Target Type Interface destructor
            */
            virtual ~TargetInterface() = default;


            // Get Target Data
            // -------------------------------
            /** \brief Get information on custom target data.
            *
            * Target data contains parameters and configuration for custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \return Return Target data [target_data::TargetData]
            */
            virtual target_data::TargetData getTargetData();


            // Update Target Data
            // -------------------------------
            /** \brief Update custom target data information.
            *
            * Target data contains parameters and configuration for custom defined target.
            * Data is initially loaded with parameter data from parameter-server
            *
            * \param target_data Updated Target data [target_data::TargetData]
            */
            virtual void updateTargetData(
                target_data::TargetData target_data);


            // Print Target Data
            // -------------------------------
            /** \brief Print information on target data to terminal.
            *
            * Implemented for debugging purposes.
            */
            virtual void printTargetData() = 0;


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Class Local Member(s)
            // -------------------------------
            target_data::TargetData target_data_;

        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:

    }; // End Class: TargetInterface
} // End Namespace: Target
#endif // TARGET_INTERFACE_H 