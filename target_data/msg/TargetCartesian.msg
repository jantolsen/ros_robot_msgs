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
                                               
# Cartesian Pose-RPY
# User-friendly representation of pose, typically used for configuration
robot_toolbox/PoseRPY pose_rpy  
                                
# Cartesian pose
# Data type used by system, typically the calculated from "pose_rpy" data         
geometry_msgs/Pose pose           
                                
                            