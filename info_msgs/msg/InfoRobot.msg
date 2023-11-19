# Info - Robot
# -------------------------------
# Description:
#   Message-type for containing information on Robot parameter data
#
# Version:
#   0.1 -   Initial version
#           [19.11.2023] - Jan T. Olsen
# -------------------------------

# Robot-type constants
int32 GP400     = 0
int32 GP215     = 1
int32 IRB6660   = 2

# Robot general parameter data
string prefix                                   # Prefix-name of robot (gets added to joint-names, links, etc.)
string type_name                                # Robot-type name [GP400, GP215, IRB6660] 
int32 type_id                                   # Robot-type ID [GP400 [0], GP215 [1], IRB6660 [2]] 
string[] joint_names                            # Joint-names of robot (joint1 [0], ..., joint6 [5])
info_msgs/InfoPlanningGroups planning_groups    # Planning-groups names for robot planning and control
info_msgs/InfoPlanningFrames planning_frames    # Robot planning-frames
info_msgs/InfoEndEffector eoat                  # Robot end-effector (EOAT) parameters 

# Robot limits parameter data
info_msgs/InfoLimitJoint[] joint_limits         # Joint limits of robot (joint1 [0], ..., joint6 [5])
info_msgs/InfoLimitCartesian cartesian_limits   # Cartesian limits of robot (translational and rotational)
