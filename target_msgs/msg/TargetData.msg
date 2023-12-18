# Target - Data
# -------------------------------
# Description:
#   Message-type for target data
#
# Version:
#   0.1 -   Initial version
#           [18.12.2023] - Jan T. Olsen
# -------------------------------

# Target type constants
int32 JOINT         = 0
int32 CARTESIAN     = 1

# Target Data
string name             # Target name
int32 id                # Target identifier
int32 type              # Target type [JOINT [0], CARTESIAN [1]]

# Target Joint
target_msgs/TargetJoint target_joint

# Target Cartesian
target_msgs/TargetCartesian target_cartesian
