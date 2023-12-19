# Robot Data - Joint-State
# -------------------------------
# Description:
#   Message-type for extended Joint-State data
#
# Version:
#   0.1 -   Initial version
#           [19.12.2023] - Jan T. Olsen
# -------------------------------

# Joint-State data
std_msgs/Header header      # Header specifying time for joint-state
string[] name               # Name of respective joints
float64[] position          # Position of respective joints
float64[] velocity          # Velocity of respective joints
float64[] acceleration      # Acceleration of respective joints
float64[] torque            # Torque of respective joints