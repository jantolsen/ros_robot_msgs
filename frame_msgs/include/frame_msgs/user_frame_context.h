// User Frame Context
// -------------------------------
// Description:
//      Robot system user frame context
//      Collects information on custom defined user frame from the parameter server. 
//      Structures and sorts the information into a user-frame message-type.
//      Provides functionality related to user-frame interaction.
//
// Version:
//  0.1 - Initial Version
//        [05.01.2024]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef USER_FRAME_CONTEXT_H       
#define USER_FRAME_CONTEXT_H

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

    // Frame Messages
    #include "frame_msgs/UserFrame.h"

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"


// User-Frame Context Class
// -------------------------------
/** \brief Robot system user frame context 
* Collects information on custom defined user frame from the parameter server.
* Structures and sorts the information into a user-frame message-type.
* Provides functionality related to user-frame interaction
*/
class UserFrameContext
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:
        // Define Shared-Pointer of Class-Object
        typedef typename std::shared_ptr<UserFrameContext> Ptr;

        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        /** \brief User-Frame Context class constuctor
        *
        * \param nh         ROS Nodehandle [ros::Nodehandle]
        * \param user_frame_data User-Frame data[frame_msgs::UserFrame]
        */
        UserFrameContext(
            ros::NodeHandle& nh,
            const frame_msgs::UserFrame& user_frame_data);


        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        /** \brief User-Frame Context class constuctor
        *
        * \param nh         ROS Nodehandle [ros::Nodehandle]
        * \param param_name User-Frame parameter name, located on parameter server [std::string]
        */
        UserFrameContext(
            ros::NodeHandle& nh,
            const std::string& param_name);

        
        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        /** \brief User-Frame Context class constuctor
        *
        * \param nh         ROS Nodehandle [ros::Nodehandle]
        * \param param_xml  User-Frame parameter, located on parameter server [XmlRpc::XmlRpcValue]
        */
        UserFrameContext(
            ros::NodeHandle& nh,
            const XmlRpc::XmlRpcValue& param_xml);


        // Class destructor
        // -------------------------------
        /** \brief User-Frame Context class destructor
        */
        ~UserFrameContext();


        // Publish User-Frame
        // -------------------------------
        /** \brief Publish user-frame. 
        *
        * Publishes custom defined user-frame as a topic.
        * The user-frame is published as a [frame_msgs::UserFrame] message-type.
        */
        void publishUserFrame();


        // Broadcast User-Frames
        // -------------------------------
        /** \brief Broadcast user-frame.
        *
        * Broadcasts custom defined user-frames as tf2 transforms.
        * The user-frame is broadcasted as a [geometry_msgs::TransformStamped] message-type.
        */
        void broadcastUserFrame();


        // Publish and Broadcast User-Frame
        // -------------------------------
        /** \brief Publish and Broadcast user-frame. 
        *
        * Publishes and broadcast custom defined user-frame.
        * The user-frame is published as a topic [frame_msgs::UserFrame].
        * and broadcasted as a tf2 transform [geometry_msgs::TransformStamped].
        */
        void publishAndBroadcastUserFrame();


        // Get User-Frame Data
        // -------------------------------
        /** \brief Get information on custom user-frame.
        *
        * User-Frame contains parameters and configuration for custom defined user-frame.
        * Data is initially loaded with parameter data from parameter-server
        *
        * \return Return User-Frame [frame_msgs::UserFrame]
        */
        frame_msgs::UserFrame getUserFrameData();


        // Update User-Frame Data
        // -------------------------------
        /** \brief Update custom user-frame information.
        *
        * User-Frame contains parameters and configuration for custom defined user-frame.
        * Data is initially loaded with parameter data from parameter-server
        *
        * \param user_frame_data Updated User-Frame [frame_msgs::UserFrame]
        */
        void updateUserFrameData(
            frame_msgs::UserFrame user_frame_data);


        // Print User-Frame
        // -------------------------------
        /** \brief Print information on custom user-frame to terminal.
        *
        * Implemented for debugging purposes.
        */
        void printUserFrame();


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Load User-Frame Parameter Data
        // -------------------------------
        // (Function Overloading)
        /** \brief Reads and loads information on custom user-frame from the parameter server.
        *
        * Organize and structure the loaded parameters into user-frame message-type.
        * If successful, the gathered user-frame is returned. 
        * Function returns fals if it fails to load user-frame data.
        *
        * \param param_name User-Frame parameter name, located on parameter server [std::string]
        * \return Function return: Successful: User-Frame data [frame_msgs::UserFrame] / Unsuccessful: false [bool]
        */
        boost::optional<frame_msgs::UserFrame> loadParamData(
            const std::string& param_name);


        // Load User-Frame Parameter Data
        // -------------------------------
        // (Function Overloading)
        /** \brief Reads and loads information on custom user-frame from the parameter server.
        *
        * Organize and structure the loaded parameters into user-frame message-type.
        * If successful, the gathered user-frame is returned. 
        * Function returns fals if it fails to load user-frame data.
        *
        * \param param_xml  User-Frame parameters [XmlRpc::XmlRpcValue]
        * \return Function return: Successful: User-Frame data [frame_msgs::UserFrame] / Unsuccessful: false [bool]
        */
        boost::optional<frame_msgs::UserFrame> loadParamData(
            const XmlRpc::XmlRpcValue& param_xml);


        // Validate Frame
        // -------------------------------
        /** \brief Validate Frame
        *
        * Validate and check if frame exists and is available.
        *
        * \param frame_name Frame name [std::string]
        * \return Function return: Successful/Unsuccessful (true/false) [bool]
        */
        bool validateFrame(
            const std::string& frame_name);


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
        // Class-Name-Prefix for terminal message
        static const std::string CLASS_PREFIX;
        
        // Class Local Member(s)
        // -------------------------------
        frame_msgs::UserFrame user_frame_data_;

        // ROS Nodehandle(s)
        // -------------------------------
        ros::NodeHandle nh_;
        
        // ROS Publsiher(s)
        // -------------------------------
        ros::Publisher user_frame_pub_;

        // ROS TF2 Broadcaster
        // -------------------------------
        tf2_ros::TransformBroadcaster tf2_broadcaster_;

}; // End Class: UserFrameContext
#endif // USER_FRAME_CONTEXT_H 