# Target - Data
# -------------------------------
# Description:
#   Message-type for target data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Type constants
int32 JOINT         = 0
int32 CARTESIAN     = 1
int32 JOINT_EXTAXIS = 2

# Target name
string name

# Target type
# Identifier for determining target-type 
# [JOINT [0], CARTESIAN [1], JOINT_EXTAXIS [2]]
int32 type
string type_name

# Target Joint
target_data/TargetJoint joint

# Target Cartesian
target_data/TargetCartesian cartesian

# Target Joint Ext-Axis
target_data/TargetJointExtAxis joint_ext_axis

# Visualize Target
bool visible