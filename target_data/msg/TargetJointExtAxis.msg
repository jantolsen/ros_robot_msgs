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

# Joint-Posiiton
# User-friendly representation of joint-poisiton, typically used for configuration
# [qE, q1, q2, q3, q4, q5, q6] ([qE] acting as external axis) 
robot_toolbox/JointPositionExtAxis joint

# Joint-Posiiton
# Data type used by system, typically the calculated from "joint" data
# [qE, q1, q2, q3, q4, q5, q6]
float64[] position

# External axis configuration
# (e.g. robot track, eoat-spindle)
target_data/InfoExtAxis external_axis