# Target - Cartesian
# -------------------------------
# Description:
#   Message-type for target cartesian data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Target Cartesian-Posiiton
string ref_frame                # Reference frame for which target pose is relative to
geometry_msgs/Pose pose         # Target cartesian pose
robot_toolbox/PoseRPY pose_rpy  # Target cartesian pose-rpy