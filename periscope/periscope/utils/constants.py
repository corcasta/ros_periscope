import numpy as np
import transforms3d as tf

# Quaternion: [w, x, y, z] 
Q_FRD_FLU = Q_FLU_FRD = [0, 1, 0, 0]                        # Transform from PX4-body to ROS-body (drone)
Q_NED_ENU = Q_ENU_NED = [0, np.sqrt(2)/2, np.sqrt(2)/2, 0]  # Transform from PX4-world to ROS-world

# Rotation representaion
R_NED_ENU = R_ENU_NED = tf.quaternions.quat2mat(Q_NED_ENU)  # Transform from PX4-body to ROS-body (drone)
R_FRD_FLU = R_FLU_FRD = tf.quaternions.quat2mat(Q_FRD_FLU)  # Transform from PX4-world to ROS-world


# Rotation representaion 6D (6X6)
R_NED_ENU_6 = R_ENU_NED_6 = R_FRD_FLU_6 = R_FLU_FRD_6 = np.zeros((6, 6))

R_NED_ENU_6[:3, :3] = R_ENU_NED_6[:3, :3] = R_NED_ENU 
R_NED_ENU_6[3:, 3:] = R_ENU_NED_6[3:, 3:] = R_NED_ENU

R_FRD_FLU_6[:3, :3] = R_FLU_FRD_6[:3, :3] = R_FRD_FLU 
R_FRD_FLU_6[3:, 3:] = R_FLU_FRD_6[3:, 3:] = R_FRD_FLU 
