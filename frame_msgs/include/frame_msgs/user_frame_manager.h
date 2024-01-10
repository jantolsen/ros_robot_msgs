// User Frame Manager
// -------------------------------
// Description:
//      Robot system user frame handler
//      Collects, sorts and structures information on custom defined user frames 
//      obtained from the parameter server. The manager utilizes the 
//      User-frame-context class to store information as a userframe-message type 
//      for each defined user-frame. Provides functionality for accessing, publishing 
//      and broadcasting information on each user-frame present in the system.
//
// Version:
//  0.1 - Initial Version
//        [08.01.2024]  -   Jan T. Olsen
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
#ifndef USER_FRAME_MANAGER_H       
#define USER_FRAME_MANAGER_H

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

    // Frame Messages
    #include "frame_msgs/UserFrame.h"

    // Frame Services
    #include "frame_msgs/CreateUserFrame.h"
    #include "frame_msgs/UpdateUserFrame.h"

    // User-Frame Context
    #include "frame_msgs/user_frame_context.h"


// Namespace: Frame
// -------------------------------
namespace Frame
{
    // User-Frame Manager Class
    // -------------------------------
    /** \brief Robot system user-frame manager
    * Collects, sorts and structures information on custom defined user frames 
    * obtained from the parameter server. The manager utilizes the 
    * User-frame-context class to store information as a userframe-message type 
    * for each defined user-frame. Provides functionality for accessing, publishing 
    * and broadcasting information on each user-frame present in the system.
    */
    class UserFrameManager
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:

            // Class constructor
            // -------------------------------
            /** \brief User-Frame Manager class constuctor
            *
            * \param nh         ROS Nodehandle [ros::Nodehandle]
            * \param param_name Parameter name for collective user-frames, located on parameter server [std::string]
            */
            UserFrameManager(
                ros::NodeHandle& nh,
                const std::string& param_name);


            // Class destructor
            // -------------------------------
            /** \brief User-Frame Manager class destructor
            */
            ~UserFrameManager();


            // Create User-Frame Object
            // -------------------------------
            // (Function Overloading)
            /** \brief Create a User-Frame object
            *
            * Creates a user-frame object [userFrameContext] from given parameter data.
            * Returns the created object as a shared-pointer
            *
            * \param user_frame_data User-Frame data (collection of parameter data) [frame_msgs::UserFrame]
            * \return Shared-Pointer of a User-Frame object [std::shared_ptr<UserFrameContext>]
            */
            std::shared_ptr<UserFrameContext> createUserFrameObject(
                const frame_msgs::UserFrame user_frame_data);


            // Create User-Frame Object
            // -------------------------------
            // (Function Overloading)
            /** \brief Create a User-Frame object
            *
            * Creates a user-frame object [userFrameContext] from given parameter data.
            * Returns the created object as a shared-pointer
            *
            * \param user_frame_param_xml User-Frame parameters [XmlRpc::XmlRpcValue]
            * \return Shared-Pointer of a User-Frame object [std::shared_ptr<UserFrameContext>]
            */
            std::shared_ptr<UserFrameContext> createUserFrameObject(
                const XmlRpc::XmlRpcValue user_frame_param_xml);


            // Get User-Frame Object
            // -------------------------------
            /** \brief Get a specific User-Frame oject
            *
            * Searches for given user-frame name in user-frame map
            * and returns the respective user-frame object.
            * If user-frame is not found within map, function returns false.
            *
            * \param user_frame_name Name of User-Frame object to locate [std:string]
            * \return Function return: Successful: User-Frame [std::shared_ptr<UserFrameContext>] / Unsuccessful: false [bool]
            */
            boost::optional<std::shared_ptr<UserFrameContext>> getUserFrameObject(
                const std::string& user_frame_name);


            // Get User-Frames Object Vector
            // -------------------------------
            /** \brief Get user-frames vector.
            *
            * Each user-frame [std::shared_ptr<UserFrameContext>] 
            * is stored in a collective vector.
            *
            * \return Return Vector of User-Frames [std::vector<std::shared_ptr<UserFrameContext>]
            */
            std::vector<std::shared_ptr<UserFrameContext>> getUserFrameObjectsVec();


            // Get User-Frames Object Map
            // -------------------------------
            /** \brief Get user-frames map
            *
            * Each user-frame [std::shared_ptr<UserFrameContext>] 
            * is paired with their respective name [std::string] 
            * and stored in a collective map.
            *
            * \return Return Map of User-Frames [std::map<std::string, std::shared_ptr<UserFrameContext>]
            */
            std::map<std::string, std::shared_ptr<UserFrameContext>> getUserObjectsMap();


            // Publish and Broadcast User-Frames
            // -------------------------------
            /** \brief Publish and Broadcast user-frames. 
            *
            * Iterate through user-frame map and publish and broadcast
            * each custom defined user-frame
            * The user-frames are published as a topic with their respective name [frame_msgs::UserFrame].
            * and broadcasted as a tf2 transform [geometry_msgs::TransformStamped].
            */
            void publishAndBroadcastUserFrames();


            // Update User-Frame Object
            // -------------------------------
            // (Function Overloading)
            /** \brief Update a specific User-Frame oject
            *
            * Searches for the given user-frame object
            * and updates the object according to the given parameter data.
            *
            * \param user_frame_oject Pointer to User-Frame object [UserFrameContext::Ptr]
            * \param user_frame_data Updated User-Frame data (collection of parameter data) [frame_msgs::UserFrame]
            */
            void updateUserFrameObject(
                UserFrameContext::Ptr& user_frame_oject,
                const frame_msgs::UserFrame& user_frame_data);


            // Update User-Frame Object
            // -------------------------------
            // (Function Overloading)
            /** \brief Update a specific User-Frame oject
            *
            * Searches for the given user-frame object
            * and updates the object according to the given parameter data.
            *
            * \param user_frame_name Name of User-Frame object to locate [std:string]
            * \param user_frame_data Updated User-Frame data (collection of parameter data) [frame_msgs::UserFrame]
            */
            void updateUserFrameObject(
                const std::string& user_frame_name,
                const frame_msgs::UserFrame& user_frame_data);


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Initialize User-Frame Manager
            // -------------------------------
            /** \brief Initialize User-Frame Manager
            */
            void init();


            // Load User-Frames Parameter Data
            // -------------------------------
            // (Function Overloading)
            /** \brief Reads and loads information on custom user-frames from the parameter server.
            *
            * Parameter information for each user-frame is stored in the local collective parameter vector
            * (param_vec_) [std::vector<XmlRpc::XmlRpcValue>].
            *
            * \param param_name User-Frames parameter name, located on parameter server [std::string]
            * \return Function return: Successful/Unsuccessful (true/false) [bool]
            */
            bool loadParamData(
                const std::string& param_name);


        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:
            // Class-Name-Prefix for terminal message
            static const std::string CLASS_PREFIX;

            // Local class member(s)
            // -------------------------------
            XmlRpc::XmlRpcValue param_name_;
            std::vector<XmlRpc::XmlRpcValue> param_vec_;
            std::map<std::string, std::shared_ptr<UserFrameContext>> user_frame_map_;
            std::vector<std::shared_ptr<UserFrameContext>> user_frame_vec_;

            // ROS Nodehandle(s)
            // -------------------------------
            ros::NodeHandle nh_;

            // ROS Service Server(s)
            // -------------------------------
            // (Declaring Service-Servers as shared-pointers to allow shared ownership of the objects)
            std::shared_ptr<ros::ServiceServer> createUserFrameObject_server_;  // ROS Server for creating a User-Frame object
            std::shared_ptr<ros::ServiceServer> updateUserFrameObject_server_;  // ROS Server for updating User-Frame object


            // Service Callback: Create User-Frame object
            // -------------------------------
            /** \brief Create User-Frame object callback function.
            *
            * Service callback function for creating a custom User-Frame object.
            * Requires information on name, reference frame and pose of the user-frame.
            *
            * \param request    Service-Request [frame_msgs::CreateUserFrame::Request]
            * \param response   Service-Response [frame_msgs::CreateUserFrame::Response]
            * \return Function result: Successful/Unsuccessful (true/false)
            */
            bool createUserFrameObjectCB(
                frame_msgs::CreateUserFrame::Request& req,
                frame_msgs::CreateUserFrame::Response& res);

            
            // Service Callback: Update User-Frame object
            // -------------------------------
            /** \brief Update User-Frame object callback function.
            *
            * Service callback function for updating a custom User-Frame object.
            * Function requires information on name of the user-frame to update.
            * The following parameters can be updated: reference frame and pose of the user-frame.
            *
            * \param request    Service-Request [frame_msgs::UpdateUserFrame::Request]
            * \param response   Service-Response [frame_msgs::UpdateUserFrame::Response]
            * \return Function result: Successful/Unsuccessful (true/false)
            */
            bool updateUserFrameObjectCB(
                frame_msgs::UpdateUserFrame::Request& req,
                frame_msgs::UpdateUserFrame::Response& res);


    }; // End Class: UserFrameManager
} // End Namespace: Frame
#endif // USER_FRAME_MANAGER_H 