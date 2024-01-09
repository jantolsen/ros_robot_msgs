# Robot Data - Joint-State
# -------------------------------
# Description:
#   Message-type for extended Joint-State data
#
# Version:
#   0.1 -   Initial version
#           [19.12.2023] - Jan T. Olsen
# -------------------------------

# Header
# Specifying time for joint-state
std_msgs/Header header

# Names of respective joints
string[] name       

# Position of respective joints
float64[] position

# Velocity of respective joints
float64[] velocity

# Acceleration of respective joints
float64[] acceleration

# Torque of respective joints
float64[] torque