# Target - Joint
# -------------------------------
# Description:
#   Message-type for target joint data
#   Normal target configuration for system with 6-joints
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
# (q1, q2, q3, q4, q5, q6) [deg or m]
robot_toolbox/JointData joint_config

# Target Joint
# Typically calculated from "joint_config" data
# (q1, q2, q3, q4, q5, q6) [rad or m]
sensor_msgs/JointState joint
