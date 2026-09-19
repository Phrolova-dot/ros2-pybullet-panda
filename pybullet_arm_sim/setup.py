from setuptools import find_packages, setup


package_name = 'pybullet_arm_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chengxunlinux',
    maintainer_email='student@example.com',
    description='PyBullet physics engine and ROS 2 adapter for the arm lab.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_node = pybullet_arm_sim.simulation_node:main',
            'camera_viewer = pybullet_arm_sim.camera_viewer:main',
        ],
    },
)
