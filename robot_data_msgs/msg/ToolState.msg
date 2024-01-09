# Robot Data - Tool-State
# -------------------------------
# Description:
#   Message-type for extended Tool-State data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Header
# Specifying time for tool-state
std_msgs/Header header

# Tool name
string name

# Frame name
# Frame for which tool pose is given relative to
string frame

# Tool Pose
robot_toolbox/PoseRPY pose

# Tool Velocity
geometry_msgs/Twist velocity

# Tool Acceleration
geometry_msgs/Accel acceleration

# Tool Torque
geometry_msgs/Wrench torque