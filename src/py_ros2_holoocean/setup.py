from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'py_ros2_holoocean'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','[!_]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Migeran Open Source',
    maintainer_email='support@migeran.com',
    description='Intial ROS2 integration for the HoloOcean simulator.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_ros2_holoocean.thruster_control_publisher:main',
            'listener = py_ros2_holoocean.simulator:main',
        ],
    },
)
