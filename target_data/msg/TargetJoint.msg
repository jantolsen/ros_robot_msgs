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

# Joint-Posiiton (deg)
# User-friendly representation of joint-poisiton, typically used for configuration
# [q1, q2, q3, q4, q5, q6]
robot_toolbox/JointPosition joint

# Joint-Posiiton (rad)
# Data type used by system, typically the calculated from "joint" data
# [q1, q2, q3, q4, q5, q6]
float64[] position
