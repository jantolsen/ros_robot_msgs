# Info - Kinematics
# -------------------------------
# Description:
#   Message-type for containing information on kinematics parameter data
#
# Version:
#   0.1 -   Initial version
#           [18.11.2023] - Jan T. Olsen
# -------------------------------

# Kinematics solver type constants
int32 KDL           = 0
int32 OPW           = 1
int32 TRACIK        = 2
int32 LMA           = 3
int32 CACHED_KDL    = 4
int32 CACHED_TRACIK = 5

# Kinematics parameter data
string solver_name          # Solver-name [KDL, OPW, TRACIK, LMA, CACHED_KDL, CACHED_TRACIK]
int32 solver_type           # Solver-type [KDL [0], OPW [1], TRACIK [2], LMA [3], CACHED_KDL [4], CACHED_TRACIK [5]]
float64 search_resolution   # Solver search-resolution
float64 timeout             # Solver timeout [s]
int32 attempts              # Solver attempts