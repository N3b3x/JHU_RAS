''' ####################
    EN605.613 - Introduction to Robotics
    Assignment 4
    Vehicle Kinematics
    -------
    For this assignment you must implement the forward and inverse kinematics functions for the Mechanum class.

    ==================================
    Copyright 2024,
    The Johns Hopkins University Whiting School of Engineering
    All Rights Reserved.
    #################### '''

import numpy as np

class Mecanum:
    """
    Kinematic implementation of a robot with 4 Mecanum wheels.
    Assume all wheels have identical sizes and pitch angles.
    Assume the body is symmetrical with the center of mass in the center.
    """

    def __init__(self,length,width,wheel_radius,roller_radius,roller_angle):
        """
        Constructor for the Mecanum kinematics class. Sets the vehicle properties.

        Input
        :param length: The length of the vehicle
        :param width: The width of the vehicle
        :param wheel_radius: The radius of the wheels
        :param wheel_radius: The radius of the wheels
        :param roller_radius: The radius of the rollers on each wheel
        :param roller_angle: The pitch angle of the rollers on each wheel

        """
        self.L = length
        self.W = width
        self.R = wheel_radius
        self.alpha = roller_angle
        self.r = roller_radius

    def forward(self, x, u):
        """
        Computes the forward kinematics for the Mecanum system.

        Input
        :param x: The starting state (position) of the system. This is [x,y,theta].
        :param u: The control input to the system. This is the wheel rates for each wheel [psi_1,psi_2,psi_3,psi_4]

        Output
        :return: v: The resulting velocity vector for the system. This is [Vx,Vy,Vtheta]
        """
        L, W, R, alpha = self.L, self.W, self.R, self.alpha
        psi1, psi2, psi3, psi4 = u

        # Updated transformation matrix incorporating the roller angle:
        T = (R / 4) * np.array([
            [np.cos(alpha),  np.cos(alpha),  np.cos(alpha),  np.cos(alpha)],
            [-np.sin(alpha), np.sin(alpha),  np.sin(alpha), -np.sin(alpha)],
            [-1/(L+W),       1/(L+W),       -1/(L+W),       1/(L+W)]
        ])

        psi = np.array([psi1, psi2, psi3, psi4])
        v = T @ psi   # Matrix multiplication computes [Vx, Vy, Vtheta]
        return v

    def inverse(self, x, v):
        """
        Computes the inverse kinematics for the Mecanum system.

        Input
        :param x: The starting state (position) of the system. This is [x,y,theta].
        :param v: The desired velocity vector for the system. This is [Vx,Vy,Vtheta]

        Output
        :return: u: The necessary control inputs to achieve the desired velocity vector. This is the wheel rates for each wheel [psi_1,psi_2,psi_3,psi_4]
        """
        L, W, R, alpha = self.L, self.W, self.R, self.alpha
        Vx, Vy, Vtheta = v

        # Compute wheel speeds by "undoing" the forward kinematics scaling:
        psi1 = (Vx / (R * np.cos(alpha))) - (Vy / (R * np.sin(alpha))) - ((L + W) * Vtheta / R)
        psi2 = (Vx / (R * np.cos(alpha))) + (Vy / (R * np.sin(alpha))) + ((L + W) * Vtheta / R)
        psi3 = (Vx / (R * np.cos(alpha))) + (Vy / (R * np.sin(alpha))) - ((L + W) * Vtheta / R)
        psi4 = (Vx / (R * np.cos(alpha))) - (Vy / (R * np.sin(alpha))) + ((L + W) * Vtheta / R)

        u = np.array([psi1, psi2, psi3, psi4])
        return u

def main():

    x0 = np.array([0,0,0])

    length = 0.3
    width =  0.15
    wheel_radius = 0.05
    roller_radius = 0.01
    roller_angle = 45/360*np.pi

    mecanum = Mecanum(length,width,wheel_radius,roller_radius,roller_angle)

    u0 = np.array([2,2,2,2])

    print(f'Simulating wheel inputs of {u0} for 3 seconds')
    dt = 0.1
    t = 0
    x = x0
    while t<3:
        v = mecanum.forward(x,u0)
        x += v * dt
        t+=dt
        print(f'{t}:{x}')

    v_desired = np.array([0,0,45/360*np.pi])
    print(f'Rotating in place with {v_desired} for 1 seconds')
    t=0
    while t<1:
        u = mecanum.inverse(x,v_desired)
        v = mecanum.forward(x,u)
        x += v * dt
        t+=dt
        print(f'{t}:{x}')

    v_desired = np.array([0.5,0.5,0])
    print(f'Translating with  {v_desired} for 2 seconds')
    t=0
    while t<2:
        u = mecanum.inverse(x,v_desired)
        v = mecanum.forward(x,u)
        x += v * dt
        t+=dt
        print(f'{t}:{x}')

if __name__ == '__main__':
    main()

