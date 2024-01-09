# Frame - User Frame
# -------------------------------
# Description:
#   Message-type for User Frame
#   Used to define a custom user frame
#
# Version:
#   0.1 -   Initial version
#           [04.01.2024] - Jan T. Olsen
# -------------------------------

# User-Frame name
string name

# Reference frame
# Frame for which user frame pose is given relative to
string ref_frame

# Pose-RPY
# User-friendly representation of pose, typically used for configuration
robot_toolbox/PoseRPY pose_rpy

# Transform-Stamped
# Data type used by tf2 broadcasting, typically calculated from "pose_rpy" data
geometry_msgs/TransformStamped transform_stamped