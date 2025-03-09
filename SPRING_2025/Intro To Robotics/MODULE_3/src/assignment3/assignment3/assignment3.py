''' ####################
    EN605.613 - Introduction to Robotics
    Assignment 3
    Kinematic Chains
    -------
    For this assignment you must implement the following functions
    1. axis_angle_rot_matrix 15 pts
    2. hr_matrix 15 pts
    3. inverse_hr_matrix 10 pts
    4. KinematicChain.pose 30 pts
    5. KinematicChain.jacobian 30 pts
    ==================================
    Copyright 2023,
    The Johns Hopkins University Whiting School of Engineering
    All Rights Reserved.
    #################### '''

import numpy as np

def axis_angle_rot_matrix(k: np.ndarray,q: float) -> np.ndarray:
    """
    Creates a 3x3 rotation matrix in 3D space from an axis and an angle.

    Input
    :param k: A 3 element array containing the unit axis to rotate around (x,y,z) 
    :param q: The angle (in radians) to rotate by

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """
    k = k / np.linalg.norm(k)  # Ensure k is a unit vector
    sin_q = np.sin(q)
    cos_q = np.cos(q)
    v = 1 - cos_q
    
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    
    R = np.eye(3) + sin_q * K + v * (K @ K)
    return R

def hr_matrix(k: np.ndarray,t: np.ndarray,q: float) -> np.ndarray:
    '''
    Create the Homogenous Representation matrix that transforms a point from Frame B to Frame A.
    Using the axis-angle representation
    Input
    :param k: A 3 element array containing the unit axis to rotate around (x,y,z) 
    :param t: The translation from frame A to frame B
    :param q: The rotation angle (i.e. joint angle)

    Output
    :return: A 4x4 Homogenous representation matrix
    '''
    R = axis_angle_rot_matrix(k, q)
    HR = np.eye(4)
    HR[:3, :3] = R
    HR[:3, 3] = t
    return HR

def inverse_hr_matrix(k: np.ndarray,t: np.ndarray,q: float) -> np.ndarray:
    '''
    Create the Inverse Homogenous Representation matrix that transforms a point from Frame A to Frame B.
    Using using the axis-angle representation
    Input
    :param k: A 3 element array containing the unit axis to rotate around (x,y,z) 
    :param t: The translation from frame A to frame B
    :param q: The rotation angle (i.e. joint angle)

    Output
    :return: A 4x4 Inverse Homogenous representation matrix
    '''
    R = axis_angle_rot_matrix(k, q).T
    t_inv = -np.dot(R, t)
    HR_inv = np.eye(4)
    HR_inv[:3, :3] = R
    HR_inv[:3, 3] = t_inv
    return HR_inv

class KinematicChain:
    def __init__(self,ks: np.ndarray,ts: np.ndarray):
        '''
        Creates a kinematic chain class for computing poses and velocities

        Input
        :param ks: A 2D array that lists the different axes of rotation (rows) for each joint
        :param ts: A 2D array that lists the translation from the previous joint to the current

        '''
        self.k = np.array(ks)
        self.t = np.array(ts)
        assert ks.shape == ts.shape, 'Warning! Improper definition of rotation axes and translations'
        self.N_joints = ks.shape[0] 

    def pose(self,Q: np.ndarray, index: int=-1, p_i: np.ndarray=np.array([0,0,0])) -> np.ndarray:
        '''
        Create compute the pose in the global frame of the joint described by the index (default is the last joint position)
        Input
        :param Q: A N element array containing the joint angles in radians
        :param p_i: A 3 element vector containing translation from the index joint to the desired point in the frame of the index joint
        :param index: The index of the joint frame being converted from (first joint is 0, the last joint is N_joints)

        Output
        :return: A 3 element vector containing the position of joint[index]
        '''
        if Q is None or len(Q) < self.N_joints:
            raise ValueError("Invalid joint angles array Q: expected size {}, got {}".format(self.N_joints, len(Q) if Q is not None else None))

        if not (-1 <= index < self.N_joints):
            raise ValueError(f"Index {index} out of bounds, must be in range [-1, {self.N_joints - 1}]")

        T = np.eye(4)
        for i in range(index + 1):
            T = T @ hr_matrix(self.k[i], self.t[i], Q[i])

        p_global = T @ np.append(p_i, 1)

        if np.isnan(p_global).any():
            raise ValueError("Pose calculation resulted in NaN values")

        result = p_global[:3]
        #print(f"pose() returning: {result}")
        return result
    
    def pseudo_inverse(self,
                        theta_start: np.ndarray,
                        p_eff_N: np.ndarray,
                        x_end: np.ndarray,
                        max_steps: int=np.inf) -> np.ndarray:
        '''
        Performs the inverse_kinematics using the pseudo-jacobian

        :param theta_start: A N element array containing the current joint angles in radians
        :param p_eff_N: A 3 element vector containing translation from the last joint to the end effector in the last joints frame of reference
        :param xend: A 3 element vector containing the desired end pose for the end effector in the base frame
        :param max_steps: (Optional) Maximum number of iterations to compute 
        (Note: If it takes more than 200 iterations it is something the computation went wrong)

        Output
        :return: An N element vector containing the joint angles that result in the end effector reaching xend
        '''
        v_step_size = 0.05
        theta_max_step = 0.2
        Q_j = theta_start
        p_end = np.array([x_end[0],x_end[1],x_end[2]])
        p_j = self.pose(Q_j,p_i=p_eff_N)
        delta_p =p_end - p_j
        j=0
        while np.linalg.norm(delta_p) > 0.01 and j<max_steps:
            print(f'j{j}: Q[{Q_j}] , P[{p_j}], d[{delta_p}]')
            v_p = delta_p * v_step_size / np.linalg.norm(delta_p)

            J_j = self.jacobian(Q_j,p_eff_N)
            J_invj = np.linalg.pinv(J_j)

            v_Q = np.matmul(J_invj,v_p)

            Q_j = Q_j + np.clip(v_Q,-1*theta_max_step,theta_max_step)#[:self.N_joints]

            p_j = self.pose(Q_j,p_i=p_eff_N)
            j+=1
            delta_p = p_end - p_j 

        return Q_j


    def jacobian(self,Q: np.ndarray,p_eff_N: np.ndarray=np.array([0,0,0])) -> np.ndarray:
        '''
        Computes the Jacobian (position portions only, not orientation)

        :param Q: A N element array containing the current joint angles in radians
        :param p_eff_N: A 3 element vector containing translation from the last joint to the end effector in the last joints frame of reference

        Output
        :return: An 3xN 2d matrix containing the jacobian matrix
        '''
        J = np.zeros((3, self.N_joints))
        T = np.eye(4)
        p_eff = self.pose(Q, p_i=p_eff_N)
        for i in range(self.N_joints):
            T = np.dot(T, hr_matrix(self.k[i], self.t[i], Q[i]))
            R = T[:3, :3]
            p = T[:3, 3]
            J[:, i] = np.cross(R @ self.k[i], p_eff - p)
        return J


def main():

    ks = np.array([[0,0,1.0],[0,1.0,0],[0,1,0.0]])
    ts = np.array([[0,0,0.0],[0,0.1,0.95],[0,0.1,0.95]])
    kc = KinematicChain(ks,ts)

    q_0 = np.array([np.pi/8,np.pi/4,np.pi/6])
    x_1 = np.array([-1.5,-0.7,0.8])

    for i in np.arange(0,kc.N_joints):
        print(kc.pose(q_0,i))

    kc.pseudo_inverse(q_0,x_1,max_steps=600)
#    kc.pseudo_inverse(q_1,x_0,max_steps=500)

if __name__ == '__main__':
    main()

