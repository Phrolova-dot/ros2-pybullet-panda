from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'pybullet_arm_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chengxunlinux',
    maintainer_email='student@example.com',
    description='Launch files and configuration for the ROS 2 PyBullet arm lab.',
    license='MIT',
    tests_require=['pytest'],
)
