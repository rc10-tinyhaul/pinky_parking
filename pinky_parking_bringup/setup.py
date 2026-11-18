from glob import glob

from setuptools import setup

package_name = 'pinky_parking_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syaseuta',
    maintainer_email='syaseuta@gmail.com',
    description='Launches the whole simulation and provides tool commands.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={'console_scripts': [
        'send_goal = pinky_parking_bringup.send_goal:main',
        'auto_explore = pinky_parking_bringup.auto_explore:main',
        'sim_speed = pinky_parking_bringup.sim_speed:main',
    ]},
)
