from setuptools import setup

package_name = 'periscope'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
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
            'video = periscope.nodes.video_sub:main',
            'drone_tf = periscope.nodes.drone_tf2_broadcaster:main'
        ],
    },
)
