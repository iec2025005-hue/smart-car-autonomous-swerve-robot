import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_car_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diya',
    maintainer_email='diya@todo.todo',
    description='Swerve drive controller and odometry node using existing swerve kinematics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swerve_controller_node = smart_car_control.swerve_controller_node:main',
            'teleop_keyboard = smart_car_control.teleop_keyboard:main',
        ],
    },
)
