# Info - Planning Frames
# -------------------------------
# Description:
#   Message-type for containing information on planning frames parameter data
#
# Version:
#   0.1 -   Initial version
#           [19.11.2023] - Jan T. Olsen
# -------------------------------

# Planning frames parameter data
string reference_frame      # Parent/reference-frame of which the robot/object is relative to
string mount_frame          # Mounting-frame of the robot/object
string base_frame           # Base-frame of the robot/object. Used for kinematic calculation 
string tool_frame           # Tool-frame of the robot/objec. Used for kinematic calculation 