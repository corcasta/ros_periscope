from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='periscope',
            namespace='', #If we add a namespace all subs topics must add the this namespace
            executable='drone_tf',
            name='drone_tf'
        ),
        Node(
            package='periscope',
            namespace='',
            executable='camera_tf',
            name='camera_tf'
        ),
        Node(
            package='periscope',
            namespace='',
            executable='sensors',
            name='drone_sensors'
        )
    ])