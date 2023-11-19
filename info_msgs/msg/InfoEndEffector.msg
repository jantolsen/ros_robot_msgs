# Info - End-Effector (EOAT)
# -------------------------------
# Description:
#   Message-type for containing information on end-effector (EOAT) parameter data
#
# Version:
#   0.1 -   Initial version
#           [19.11.2023] - Jan T. Olsen
# -------------------------------

# End-Effector (EOAT) general parameter data
bool installed                  # End-Effector is installed (true/false)
string name                     # End-Effector name
string mesh_file_path           # File-Path to end-effector mesh STL-file

# End-Effector (EOAT) Tool-Center-Point (TCP) offset
geometry_msgs/Pose tcp_offset   # Pose offset of EOAT TCP relative to robot interface-plate