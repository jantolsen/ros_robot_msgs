# Target - Cartesian
# -------------------------------
# Description:
#   Message-type for target cartesian data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Target name
string name

# Reference frame 
# Frame for which target pose is given relative to
string ref_frame                              
                                               
# Target Cartesian Pose-RPY Configuration
# User-friendly representation of pose
robot_toolbox/PoseRPY pose_config  

# Target Cartesian Pose
# Typically calculated from "pose_configuration" data
geometry_msgs/Pose pose 
                                
                            