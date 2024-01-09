# Target - Joint
# -------------------------------
# Description:
#   Message-type for target joint data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Joint-Posiiton (deg)
# User-friendly representation of joint-poisiton, typically used for configuration
# [q0, q1, q2, q3, q4, q5, q6] 
float64[] position_deg

# Joint-Posiiton (rad)
# Data type used by system, typically the calculated from "pose_rpy" data
# [q0, q1, q2, q3, q4, q5, q6] 
float64[] position
