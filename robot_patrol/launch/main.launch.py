from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable ='direction_service_node',
            output ='screen',
            emulate_tty=True
        ),

        Node(
            package='robot_patrol',
            executable ='patrol_with_service_node',
            output ='screen',
            emulate_tty=True
        )
    ])