#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from example_interfaces.srv import Trigger

from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time

class DiffDriveSim(Node):
    def __init__(self):
        super().__init__('diffdrive_sim')

        # Parameters for a differential-drive
        self.track_width = 0.825    # distance between left and right wheels (meters)
        self.wheel_radius = 0.4     # radius of the wheels (meters)

        # Robot pose [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel angles [left, right] in radians
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Store commanded velocities
        self.lin_vel = 0.0   # m/s
        self.ang_vel = 0.0   # rad/s

        # Create subscriber for /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publisher for /joint_states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Create a TransformBroadcaster for publishing odom → chassis
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a service for resetting pose: /robot_pose_reset
        self.reset_srv = self.create_service(Trigger, '/robot_pose_reset', self.handle_reset)

        # Create a timer to update state at 30 Hz
        self.timer_period = 1.0 / 30.0
        self.timer = self.create_timer(self.timer_period, self.update_state)

        self.get_logger().info("DiffDriveSim node has been started.")

    def cmd_vel_callback(self, msg: Twist):
        """
        Receive linear and angular velocities from /cmd_vel.
        """
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z

    def handle_reset(self, request, response):
        """
        Service callback to reset the robot pose and wheel angles.
        """
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Return success
        response.success = True
        response.message = ""  # empty string
        return response

    def update_state(self):
        """
        30 Hz update loop. Computes new pose, updates wheel angles,
        publishes /joint_states, and publishes TF transform.
        """
        dt = self.timer_period

        # Differential drive kinematics
        # Based on the commanded linear velocity and angular velocity
        v_left = self.lin_vel - (self.ang_vel * self.track_width / 2.0)
        v_right = self.lin_vel + (self.ang_vel * self.track_width / 2.0)

        # Update wheel angles (radians)
        self.left_wheel_pos += (v_left / self.wheel_radius) * dt
        self.right_wheel_pos += (v_right / self.wheel_radius) * dt

        # Average linear velocity of the robot
        v = (v_left + v_right) / 2.0
        # Angular velocity of the robot
        w = (v_right - v_left) / self.track_width

        # Integrate to get new pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Publish the joint states
        self.publish_joint_states()

        # Publish the odom -> chassis TF
        self.publish_tf()

    def publish_joint_states(self):
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ["chassis_to_left_wheel", "chassis_to_right_wheel"]
        js_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        # velocity etc. optional
        self.joint_state_pub.publish(js_msg)

    def publish_tf(self):
        """
        Publish the transform from 'odom' frame to 'chassis' frame
        using the current x, y, theta of the robot.
        """
        t = TransformStamped()

        # We can consider "odom" as the world frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "chassis"

        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Rotation about Z axis
        # Convert theta (Euler) to quaternion
        # (roll=0, pitch=0, yaw=theta)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
