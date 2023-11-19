# Info - General
# -------------------------------
# Description:
#   Message-type for containing information on general parameter data
#
# Version:
#   0.1 -   Initial version
#           [18.11.2023] - Jan T. Olsen
# -------------------------------

# General parameter data
bool simulation         # Enabled/disabled simulation (true/false)
int32 robot_count       # Number of installed robot manipulators
string global_frame     # Global reference frame

# Kinematics parameter data
info_msgs/InfoKinematics kinematics # Information on kinematics parameter data