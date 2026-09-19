from setuptools import find_packages, setup


package_name = 'pybullet_arm_control'

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
    description='Trajectory action server and demos for the PyBullet arm lab.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_controller = pybullet_arm_control.trajectory_controller:main',
            'demo_sequence = pybullet_arm_control.demo_sequence:main',
            'ik_demo = pybullet_arm_control.ik_demo:main',
            'pick_place = pybullet_arm_control.pick_place:main',
            'auto_sort = pybullet_arm_control.auto_sort:main',
        ],
    },
)
