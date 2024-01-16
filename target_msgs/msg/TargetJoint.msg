# Target - Joint
# -------------------------------
# Description:
#   Message-type for target joint data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Target name
string name

# Number of Joints
# Normal target configuration: 6-joints
# Special target configuration: 7-joints (includes external axis e.g robot-track)
int32 num_joints

# Joint-Posiiton (deg)
# User-friendly representation of joint-poisiton, typically used for configuration
# [q1, q2, q3, q4, q5, q6] ([q0] is included with external axis configuration) 
float64[] position_deg

# Joint-Posiiton (rad)
# Data type used by system, typically the calculated from "pose_rpy" data
# [q1, q2, q3, q4, q5, q6] ([q0] is included with external axis configuration)
float64[] position

# External axis
# Robot configuration for external axis (e.g. robot track)
target_msgs/TargetExtAxis external_axis
