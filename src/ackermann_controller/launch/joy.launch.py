from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackermann_controller',
            executable='ackermann_teleop_joy',
            parameters=[
                {'max_speed': 1},
                {'max_steering_angle': 1}
            ]
        ),
        Node(
            package='joy',
            executable='joy_node'
        )
    ])