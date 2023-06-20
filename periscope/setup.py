import os
from glob import glob
from setuptools import setup

package_name = 'periscope'

setup(
    name=package_name,
    version='2.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    package_data={package_name: ['siyi_sdk/*']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='corcasta',
    maintainer_email='corcasta@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = periscope.nodes.demo:main',
            'stalker = periscope.nodes.stalker:main',
            'camera_controller = periscope.nodes.camera_controller:main',
            'video = periscope.nodes.video_sub:main',
            'drone_odometry = periscope.nodes.px4_odometry_transform:main',
            'gimbal_orientation = periscope.nodes.gimbal_tf2_broadcaster:main',
            'camera_orientation = periscope.nodes.camera_tf2_broadcaster:main',
        ],
    },
)
