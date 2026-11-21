from glob import glob
import os

from setuptools import setup

package_name = 'pinky_parking_gazebo'


def tree(folder):
    """Install every file under folder into share, keeping the folder layout."""
    entries = {}
    for path in glob(f'{folder}/**/*', recursive=True):
        if os.path.isfile(path):
            target = os.path.join('share', package_name, os.path.dirname(path))
            entries.setdefault(target, []).append(path)
    return sorted(entries.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ] + tree('models'),
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
    entry_points={
        'console_scripts': [
            'marker_check = pinky_parking_gazebo.marker_check:main',
        ],
    },
)
