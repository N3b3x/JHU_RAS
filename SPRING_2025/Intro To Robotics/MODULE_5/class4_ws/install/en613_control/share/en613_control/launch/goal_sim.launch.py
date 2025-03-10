import os
from launch import LaunchDescription
import launch.actions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package share directory
    pkg_share = FindPackageShare('en613_control').find('en613_control')

    # Define the path to the URDF file in the package
    urdf_path = os.path.join(pkg_share, 'urdf', 'basic_robot.urdf')
    rviz_file = os.path.join(pkg_share, 'launch', 'dummy_model.rviz')

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Return the launch description with the necessary nodes
    return LaunchDescription([
        # Node to publish the robot state (publishes TFs from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Node to start the differential drive simulator
        Node(
            package='en613_control',
            executable='diffdrive_sim',
            name='diffdrive_sim',
            output='screen'
        ),
        # Node to start the PID controller
        Node(
            package='en613_control',
            executable='diffdrive_pid',
            name='diffdrive_pid',
            output='screen'
        ),
        launch.actions.ExecuteProcess(cmd=['rviz2','-d',rviz_file])
    ])
