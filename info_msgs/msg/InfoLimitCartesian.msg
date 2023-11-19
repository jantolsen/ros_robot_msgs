# Info - Joint Limit
# -------------------------------
# Description:
#   Message-type for containing information on cartesian limits parameter data
#
# Version:
#   0.1 -   Initial version
#           [19.11.2023] - Jan T. Olsen
# -------------------------------

# Cartesian limit parameter data
info_msgs/InfoLimit trans_pos       # Translational position limits [m]
info_msgs/InfoLimit rot_pos         # Rotational position limits [rad]
info_msgs/InfoLimit trans_vel       # Translational velocity limits [m/s]
info_msgs/InfoLimit rot_vel         # Rotational velocity limits [rad/s]
info_msgs/InfoLimit trans_acc       # Translational acceleration limits [m/s^2]
info_msgs/InfoLimit rot_acc         # Rotational acceleration limits [rad/s^2]
