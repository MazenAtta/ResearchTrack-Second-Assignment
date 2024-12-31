from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_urdf',  # Replace with your actual package name
            executable='move_robot_node.py',  # Ensure the file is executable
            name='move_robot_node',
            output='screen',
            parameters=[]
        ),
    ])
