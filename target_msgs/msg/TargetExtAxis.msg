# Target - External Axis
# -------------------------------
# Description:
#   Message-type for target external-axis data
#   This types complements the target joint data
#
# Version:
#   0.1 -   Initial version
#           [14.01.2024] - Jan T. Olsen
# -------------------------------

# Exteterna Axis Type constants
int32 ROTATION  = 0
int32 LINEAR    = 1

# External axis
# Target configuration includes external axis (e.g. robot track)
bool installed

# Identifier for determining axis-type 
# [ROTATION [0], LINEAR [1]]
int32 axis_type
string axis_type_name

