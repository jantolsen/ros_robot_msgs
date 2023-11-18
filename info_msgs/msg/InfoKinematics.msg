# Info - Kinematics
# -------------------------------
# Message-type for containing information on kinematics parameter data

# Kinematics solver type constants
int32 KDL           = 0
int32 OPW           = 1
int32 TRACIK        = 2
int32 LMA           = 3
int32 CACHED_KDL    = 4
int32 CACHED_TRACIK = 5

# Kinematics parameter data
string solver               # Solver-name [KDL, OPW, TRACIK, LMA, CACHED_KDL, CACHED_TRACIK]
int32 solver_id             # Solver-ID [KDL [0], OPW [1], TRACIK [2], LMA [3], CACHED_KDL [4], CACHED_TRACIK [5]]
float64 search_resolution   # Solver search-resolution
float64 timeout             # Solver timeout [s]
float64 attempts            # Solver attempts