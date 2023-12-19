# Robot Data - Tool-State
# -------------------------------
# Description:
#   Message-type for extended Tool-State data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Tool-State data
std_msgs/Header header              # Header specifying time for joint-state
string name                         # Name of tool
string frame                        # Frame of tool
robot_toolbox/PoseRPY pose          # Pose of tool
geometry_msgs/Twist velocity        # Velocity of tool
geometry_msgs/Twist acceleration    # Acceleration of tool
geometry_msgs/Wrench torque         # Torque of tool