#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np
from .assignment4 import Mecanum  # Adjust this import if your package structure is different

class MecanumSim(Node):
    def __init__(self):
        super().__init__('mecanum_sim')
        # Initialize publishers for each wheel command
        self.pub_fl = self.create_publisher(Float64, '/fl_wheel_vel_cmd', 10)
        self.pub_fr = self.create_publisher(Float64, '/fr_wheel_vel_cmd', 10)
        self.pub_bl = self.create_publisher(Float64, '/bl_wheel_vel_cmd', 10)
        self.pub_br = self.create_publisher(Float64, '/br_wheel_vel_cmd', 10)

        # Subscriber for the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        # Prevent unused variable warning
        self.subscription

        # Create an instance of your Mecanum class with required arguments
        length = 0.3
        width = 0.15
        wheel_radius = 0.05
        roller_radius = 0.01
        roller_angle = 45 / 360 * np.pi
        self.mecanum = Mecanum(length, width, wheel_radius, roller_radius, roller_angle)

    def cmd_vel_callback(self, msg: Twist):
        # Compute wheel velocities using inverse kinematics:
        # The first argument ([0,0,0]) is a dummy state value if it's not used.
        fl, fr, bl, br = self.mecanum.inverse([0, 0, 0], [msg.linear.x, msg.linear.y, msg.angular.z])
        
        # Publish each wheel velocity as a Float64 message
        self.pub_fl.publish(Float64(data=fl))
        self.pub_fr.publish(Float64(data=fr))
        self.pub_bl.publish(Float64(data=bl))
        self.pub_br.publish(Float64(data=br))
        
        self.get_logger().info(
            f'cmd_vel received: linear_x: {msg.linear.x}, linear_y: {msg.linear.y}, angular_z: {msg.angular.z} | '
            f'Wheel Velocities: FL: {fl}, FR: {fr}, BL: {bl}, BR: {br}')

def main(args=None):
    rclpy.init(args=args)
    node = MecanumSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
