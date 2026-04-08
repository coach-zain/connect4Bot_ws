from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3e_moveit_control',
            executable='pick_and_place',
            name='ur3e_pick_and_place',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ],
        )
    ])