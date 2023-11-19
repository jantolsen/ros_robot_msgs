# Info - Kinematics OPW
# -------------------------------
# Description:
#   Message-type for containing information on opw kinematics parameter data
#   (see https://github.com/jantolsen/ros_opw_kinematics for more details)
#
# Version:
#   0.1 -   Initial version
#           [19.11.2023] - Jan T. Olsen
# -------------------------------

# OPW Kinematics - Geometric parameter data
float64 a1
float64 a2
float64 b
float64 c1
float64 c2
float64 c3
float64 c4

# OPW Kinematics - Joint offsets
float64[6] joint_offset         # Joint-offset vector (joint1 [0], ..., joint6 [5]) [rad]
int32[6] joint_sign_correction  # Joint-sign-correct vector (joint1 [0], ..., joint6 [5]) [-1 or 1]