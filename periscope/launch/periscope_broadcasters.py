from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='periscope',
            namespace='broadcasters',
            executable='drone_tf',
            name='drone_tf'
        ),
        Node(
            package='periscope',
            namespace='broadcasters',
            executable='camera_tf',
            name='camera_tf'
        )
    ])