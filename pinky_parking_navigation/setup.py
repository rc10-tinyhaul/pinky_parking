from glob import glob

from setuptools import setup

package_name = 'pinky_parking_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'odom_localizer = pinky_parking_navigation.odom_localizer:main',
            'tf_pose = pinky_parking_navigation.tf_pose:main',
        ],
    },
    maintainer='syaseuta',
    maintainer_email='syaseuta@gmail.com',
    description='Nav2 per robot domain on a shared map.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
