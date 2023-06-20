from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='periscope',
            namespace='',
            executable='drone_odometry',
            name='drone_odometry'
        ),
        Node(
            package='periscope',
            namespace='',
            executable='gimbal_orientation',
            name='gimbal_orientation'
        ),
        Node(
            package='periscope',
            namespace='',
            executable='camera_orientation',
            name='camera_orientation'
        ),
        Node(
            package='periscope',
            namespace='',
            executable='camera_controller',
            name='camera_controller'
        ),
        Node(
            package='periscope',
            namespace='',
            executable='stalker',
            name='stalker'
        ),
        Node(
            package='periscope',
            namespace='',
            executable='video',
            name='video'
        )
    ])