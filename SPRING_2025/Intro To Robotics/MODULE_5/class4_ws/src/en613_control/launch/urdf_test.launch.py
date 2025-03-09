import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package share directory
    pkg_share = FindPackageShare('en613_control').find('en613_control')

    # Define the path to the URDF file in the package
    urdf_path = os.path.join(pkg_share, 'urdf', 'basic_robot.urdf')

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Return the launch description with the necessary nodes
    return LaunchDescription([
        # Node to publish the robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Node to publish the joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        # ExecuteProcess to launch RViz without a config file
        ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        )
    ])
