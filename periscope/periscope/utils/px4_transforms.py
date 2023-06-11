import numpy as np
import transforms3d as tf
from .constants import * #import Q_ENU_NED, Q_NED_ENU, Q_FRD_FLU, Q_FLU_FRD 


def transform_orientation(q, transformation):
    """
    Transforms a quaternion vector from base frame to target frame.  
    
    Args:
            q (list, np.array): List/Array of size 4 (the elements of the quaternion must 
                                follow this order [x, y, z, w]).
                                Shape: (4,1), (1,4), (4,), (,4).
                                  
            transformation (str): Choose one of the body or world transformations available.
                                  BODY:
                                      FLU_2_FRD
                                      FRD_2_FLU
  
                                  WORLD:
                                      NED_2_ENU
                                      ENU_2_NED
    
    Returns:
        np.array: Transformed quaternion [x, y, z, w]. Shape (,4)
    """

        # Quaternion: [w, x, y, z] 
        q = np.array([q[-1], q[0], q[1], q[2]])
        
        match transformation:
            case "FRD_2_FLU":
                # Quaternion: [w, x, y, z]
                q_out = tf.quaternions.qmult(Q_FRD_FLU, q)
                
            case "FLU_2_FRD":
                # Quaternion: [w, x, y, z]
                q_out = tf.quaternions.qmult(Q_FLU_FRD, q)
                
            case "NED_2_ENU":
                # Quaternion: [w, x, y, z] 
                q_out = tf.quaternions.qmult(q, Q_NED_ENU)
                
            case "ENU_2_NED":
                # Quaternion: [w, x, y, z] 
                q_out = tf.quaternions.qmult(q, Q_ENU_NED)
                
        # Quaternion: [x, y, z, w] 
        q_out = [q_out[1], q_out[2], q_out[3], q_out[0]]    
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
        np.array: Transformed vector. Shape (,3)
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


