import numpy as np
import transforms3d as tf
from .constants import *


def transform_orientation(q, transformation):
    """
    Transforms a quaternion vector from base frame to target frame.  
    
    Args:
            q (list, np.array): List/Array of size 4 (the elements of the quaternion must 
                                follow this order [w, x, y, z).
                                Shape: (4,1), (1,4), (4,), (,4).
                                  
            transformation (str): Choose one of the body or world transformations available.
                                  BODY:
                                      FLU_2_FRD
                                      FRD_2_FLU
  
                                  WORLD:
                                      NED_2_ENU
                                      ENU_2_NED
    
    Returns:
        np.array: Transformed quaternion [w, x, y, z,]. Shape (,4)
    """
   
    match transformation:
        case "FRD_2_FLU":
            # Quaternion: [w, x, y, z]
            q_out = tf.quaternions.qmult(q, Q_FRD_FLU)
            
        case "FLU_2_FRD":
            # Quaternion: [w, x, y, z]
            q_out = tf.quaternions.qmult(q, Q_FLU_FRD)
            
        case "NED_2_ENU":
            # Quaternion: [w, x, y, z]
            q_out = tf.quaternions.qmult(Q_NED_ENU, q)
            
        case "ENU_2_NED":
            # Quaternion: [w, x, y, z]
            q_out = tf.quaternions.qmult(Q_ENU_NED, q)
           
    return q_out
    
    
def transform_position(vector, transformation):
    """
    Transforms a 3D position vector from base frame to target frame.  
    
    Args:
            vector (list, np.array): List/Array of size 3.
                                     Shape: (3,1), (1,3), (3,), (,3).
                                  
            transformation (str): Choose one of the body or world transformations available.
                                  BODY:
                                      FLU_2_FRD
                                      FRD_2_FLU
  
                                  WORLD:
                                      NED_2_ENU
                                      ENU_2_NED
    
    Returns:
        np.array: Transformed vector [x, y, z]. Shape (,3)
    """

    match transformation:
        case "FRD_2_FLU":
            vector_out = tf.quaternions.rotate_vector(vector, Q_FRD_FLU)
            
        case "FLU_2_FRD":
            vector_out = tf.quaternions.rotate_vector(vector, Q_FLU_FRD)
            
        case "NED_2_ENU":
            vector_out = tf.quaternions.rotate_vector(vector, Q_NED_ENU)
            
        case "ENU_2_NED": 
            vector_out = tf.quaternions.rotate_vector(vector, Q_ENU_NED)
            
    return vector_out
 
 
def transform_cov3d(cov, transformation):
    """
    Transforms a 3D covariance matrix from base frame to target frame.  
    
    Args:
            cov (list, np.array): Covariance matrix of size 3 (3x3).
                                  
            transformation (str): Choose one of the body or world transformations available.
                                  BODY:
                                      FLU_2_FRD
                                      FRD_2_FLU
  
                                  WORLD:
                                      NED_2_ENU
                                      ENU_2_NED
    
    Returns:
        np.array: Transformed covariance 3D (3x3)
    """
    
    match transformation:
        case "FRD_2_FLU":
            cov_out = R_FRD_FLU @ cov @ R_FRD_FLU.T
            
        case "FLU_2_FRD":
            cov_out = R_FLU_FRD @ cov @ R_FLU_FRD.T
            
        case "NED_2_ENU":
            cov_out = R_NED_ENU @ cov @ R_NED_ENU.T
            
            
        case "ENU_2_NED":
            cov_out = R_ENU_NED @ cov @ R_ENU_NED.T
            
    return cov_out     


def transform_cov6d(cov, transformation):
    """
    Transforms a 6D covariance matrix from base frame to target frame. 
    
    Args:
            cov (list, np.array): Covariance matrix of size 6 (6x6)
                                  
            transformation (str): Choose one of the body or world transformations available.
                                  BODY:
                                      FLU_2_FRD
                                      FRD_2_FLU
  
                                  WORLD:
                                      NED_2_ENU
                                      ENU_2_NED
    
    Returns:
        np.array: Transformed covariance 6D (6x6).
    """
    
    match transformation:
        case "FRD_2_FLU":
            cov_out = R_FRD_FLU_6 @ cov @ R_FRD_FLU_6.T
            
        case "FLU_2_FRD":
            cov_out = R_FLU_FRD_6 @ cov @ R_FLU_FRD_6.T
            
        case "NED_2_ENU":
            cov_out = R_NED_ENU_6 @ cov @ R_NED_ENU_6.T
            
        case "ENU_2_NED":
            cov_out = R_ENU_NED_6 @ cov @ R_ENU_NED_6.T
            
    return cov_out     


def qinverse(q):
    q_inv = tf.quaternions.qinverse(q)
    return q_inv


def px4_to_ros_orientation(q):
    """
    Transforms quaternion provided from PX4 convention to ROS convention.
    (The FRD (NED) conventions are adopted on all PX4 topics unless explicitly 
    specified in the associated message definition. ROS follows the FLU (ENU) 
    convention) 

    Args:
        q (np.array, list): Quaternion provided from any PX4 topic following 
                            FRD (NED) conventions. 
                            Order: [w, x, y, z]

    Returns:
        np.array: Quaternion from FLU to ENU.
                  Order: [w, x, y, z]
    """
    
    q_out = transform_orientation(transform_orientation(q, "ENU_2_NED"), "FRD_2_FLU")
    return q_out


def ros_to_px4_orientation(q):
    """
    Transforms quaternion from ROS convention to PX4.
    (The FRD (NED) conventions are adopted on all PX4 topics unless explicitly 
    specified in the associated message definition. ROS follows the FLU (ENU) 
    convention)

    Args:
        q (np.array, list): Quaternion provided from ROS FLU (ENU) conventions. 
                            Order: [w, x, y, z]

    Returns:
        np.array: Quaternion from FRD to NED.
                  Order: [w, x, y, z]
    """
    
    q_out = transform_orientation(transform_orientation(q, "NED_2_ENU"), "FLU_2_FRD")
    return q_out
