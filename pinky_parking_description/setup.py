from glob import glob

from setuptools import setup

package_name = 'pinky_parking_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syaseuta',
    maintainer_email='syaseuta@gmail.com',
    description='Pinky Pro vehicle models: the official robot with an optional rigid trailer.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
