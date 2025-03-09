
''' ####################
    EN605.613 - Introduction to Robotics
    Assignment 2
    Coordinate Transforms
    -------
    For this assignment you must implement the all the functions in this file
    ==================================
    Copyright 2022,
    The Johns Hopkins University Applied Physics Laboratory LLC (JHU/APL).
    All Rights Reserved.
    #################### '''

from typing import Tuple
import numpy as np


def euler_rotation_matrix(alpha: float,beta: float,gamma:float) -> np.ndarray:
    """
    15 pts
    Creates a 3x3 rotation matrix in 3D space from euler angles

    Input
    :param alpha: The roll angle (radians)
    :param beta: The pitch angle (radians)
    :param gamma: The yaw angle (radians)

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """
    
    # It may seem redundant but you can substantially increase the speed of computing
    # the rotation by precomputing the sine and cosine values

    ca, cb, cg = np.cos(alpha), np.cos(beta), np.cos(gamma)
    sa, sb, sg = np.sin(alpha), np.sin(beta), np.sin(gamma)
    
    Rz = np.array([[cg, -sg, 0], [sg, cg, 0], [0, 0, 1]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    
    R = Rz @ Ry @ Rx
    return R

def quaternion_rotation_matrix(Q: np.ndarray) -> np.ndarray:
    """
    15 pts
    Creates a 3x3 rotation matrix in 3D space from a quaternion.

    Input
    :param q: A 4 element array containing the quaternion (q0,q1,q2,q3) 

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """
    q0, q1, q2, q3 = Q
    
    R = np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])
    return R

def quaternion_multiply(Q0: np.ndarray,Q1: np.ndarray) -> np.ndarray:
    """
    15 pts
    Multiplies two quaternions.

    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 

    """
    q0, q1, q2, q3 = Q0
    r0, r1, r2, r3 = Q1
    Q = np.array([
        q0*r0 - q1*r1 - q2*r2 - q3*r3,
        q0*r1 + q1*r0 + q2*r3 - q3*r2,
        q0*r2 - q1*r3 + q2*r0 + q3*r1,
        q0*r3 + q1*r2 - q2*r1 + q3*r0
    ])
    return Q


def quaternion_to_euler(Q: np.ndarray) -> np.ndarray:
    """
    15 pts
    Takes a quaternion and returns the roll, pitch yaw array.

    Input
    :param Q0: A 4 element array containing the quaternion (q01,q11,q21,q31) 

    Output
    :return: A 3 element array containing the roll,pitch, and yaw (alpha,beta,gamma) 

    """
    q0, q1, q2, q3 = Q
    
    roll = np.arctan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1**2 + q2**2))
    pitch = np.arcsin(2 * (q0*q2 - q3*q1))
    yaw = np.arctan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2**2 + q3**2))
    
    return np.array([roll, pitch, yaw])


def rotate(p1: np.ndarray,alpha: float,beta: float,gamma: float) -> np.ndarray:
    """
    15 pts
    Rotates a point p1 in 3D space to a new coordinate system.

    Input
    :param p1: A 3 element array containing the original (x1,y2,z1) position]
    :param alpha: The roll angle (radians)
    :param beta: The pitch angle (radians)
    :param gamma: The yaw angle (radians)

    Output
    :return: A 3 element array containing the new rotated position (x2,y2,z2)

    """
    R = euler_rotation_matrix(alpha, beta, gamma)
    return R @ p1


def inverse_rotation(p2: np.ndarray,alpha: float,beta: float,gamma: float) -> np.ndarray:
    """
    15 pts
    Inverse rotation from a point p2 in 3D space to the original coordinate system.

    Input
    :param p: A 3 element array containing the new rotated position (x2,y2,z2)
    :param alpha: The roll angle (radians)
    :param beta: The pitch angle (radians)
    :param gamma: The yaw angle (radians)

    Output
    :return: A 3 element array containing the original (x1,y1,z1) position]

    """
    R = euler_rotation_matrix(alpha, beta, gamma)
    return np.linalg.inv(R) @ p2

def transform_pose(P: np.ndarray,Q: np.ndarray,T: np.ndarray,R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    10 pts
    Takes a position and orientation in the original frame along with a translation and
    rotation. 

    Then converts the original point into the new coordinate system.

    Hints: 
    - Compute the quaternion that represents the new orientation by multiplying the
      old quaternion by the new quaternion (order matters!)
    - When transforming the point rotation is applied before translation

    Input
    :param P: A 3 element array containing the position (x0,y0,z0) in the original frame
    :param Q: A 4 element array containing the quaternion (q0,q1,q2,q3) 
    :param T: A 3 element array containing the vector between the origins in the two coordinate systems (dx,dy,dz) 
    :param R: A 4 element array containing the rotation in the form of a quaternion (q0,q1,q2,q3) 

    Output
    :return: New Pose, A 3 element array (x1,y2,z1) containing the position in the new coordinate frame
    :return: New Quaternion, A 4 element array containing the orientation (q0,q1,q2,q3) in the new coordinate frame

    """
    try:
        R_matrix = quaternion_rotation_matrix(R)
        new_position = R_matrix @ P + T
        new_quaternion = quaternion_multiply(R, Q)
        return new_position, new_quaternion
    except Exception as e:
        print(f"Error in transform_pose: {e}")
        return None, None  # Ensures a tuple is always returned




