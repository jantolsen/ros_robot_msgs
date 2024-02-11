# Target - Joint
# -------------------------------
# Description:
#   Message-type for target joint data with external axis
#   E.g. robot-track, eoat-spindle
#   Special target configuration for system with 7-joints
#   (6-joints for robot + 1-joint for external axis)
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Target name
string name

# Number of Joints
int32 num_joints

# Target Joint Configuration
# User-friendly representation of joint-data
# (qE, q1, q2, q3, q4, q5, q6) [deg or m]
# ([qE] acting as external axis) 
robot_toolbox/JointDataExtAxis joint_config

# Target Joint
# Data type used by system, typically the calculated from "joint" data
# [qE, q1, q2, q3, q4, q5, q6] [rad or m]
# ([qE] acting as external axis) 
sensor_msgs/JointState joint

# External axis configuration
# (e.g. robot track, eoat-spindle)
target_data/InfoExtAxis external_axis
