#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion (x, y, z, w) to yaw angle.
    """
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class DiffDrivePID(Node):
    def __init__(self):
        super().__init__('diffdrive_pid')

        # Declare and retrieve PID parameters
        self.Kp_lin = self.declare_parameter('Kp_lin', 1.0).value
        self.Kd_lin = self.declare_parameter('Kd_lin', 0.0).value
        self.Kp_ang = self.declare_parameter('Kp_ang', 2.0).value
        self.Kd_ang = self.declare_parameter('Kd_ang', 0.0).value

        # Initialize goal pose
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0

        # Initialize previous errors and time
        self.prev_lin_err = 0.0
        self.prev_ang_err = 0.0
        self.prev_time = self.get_clock().now()

        # Initialize tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop at 30 Hz
        self.create_timer(1.0 / 30.0, self.control_loop)

        self.get_logger().info("DiffDrivePID node started.")

    def goal_pose_callback(self, msg: PoseStamped):
        """
        Callback to update the goal pose.
        """
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_theta = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}, theta={self.goal_theta:.2f}")

    def control_loop(self):
        """
        Main control loop to compute and publish velocity commands.
        """
        try:
            # Lookup the latest transform from 'odom' to 'chassis'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'chassis', now, Duration(seconds=1.0))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            return

        # Extract current position and orientation
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Compute distance and angle errors
        dx = self.goal_x - x
        dy = self.goal_y - y
        distance_error = math.sqrt(dx * dx + dy * dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Compute time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        dt = max(dt, 1e-9)  # Prevent division by zero

        # PID control for linear velocity
        lin_p = self.Kp_lin * distance_error
        lin_d = self.Kd_lin * (distance_error - self.prev_lin_err) / dt
        lin_cmd = lin_p + lin_d

        # PID control for angular velocity
        ang_p = self.Kp_ang * yaw_error
        ang_d = self.Kd_ang * (yaw_error - self.prev_ang_err) / dt
        ang_cmd = ang_p + ang_d

        # Limit commands to maximum velocities
        max_lin_speed = 1.0
        max_ang_speed = 1.0
        lin_cmd = max(-max_lin_speed, min(max_lin_speed, lin_cmd))
        ang_cmd = max(-max_ang_speed, min(max_ang_speed, ang_cmd))

        # If close to the goal, adjust orientation
        if distance_error < 0.05:
            lin_cmd = 0.0
            yaw_error = self.goal_theta - yaw
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
            ang_p = self.Kp_ang * yaw_error
            ang_d = self.Kd_ang * (yaw_error - self.prev_ang_err) / dt
            ang_cmd = ang_p + ang_d
            ang_cmd = max(-max_ang_speed, min(max_ang_speed, ang_cmd))

        # Publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = lin_cmd
        twist_msg.angular.z = ang_cmd
        self.cmd_pub.publish(twist_msg)

        # Update previous errors and time
        self.prev_lin_err = distance_error
        self.prev_ang_err = yaw_error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
