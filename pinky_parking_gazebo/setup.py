from glob import glob

from setuptools import setup

package_name = 'pinky_parking_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syaseuta',
    maintainer_email='syaseuta@gmail.com',
    description='Gazebo world and spawning of Pinky Pro robots, one ROS domain per robot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={'console_scripts': [
        'make_lane_world = pinky_parking_gazebo.lane_track:main',
        'track_check = pinky_parking_gazebo.track_check:main',
    ]},
)
