from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ardupilot_gui',
            executable='ardupilot_gui',
            name='ardupilot_gui_node',
            output='screen',
            parameters=[]
        )
    ])