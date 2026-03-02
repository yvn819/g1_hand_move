from glob import glob
from setuptools import find_packages, setup

package_name = 'g1_arm_joint_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yutong',
    maintainer_email='yutong@example.com',
    description='ROS 2 package for commanding Unitree G1 arm joints with target angles.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'arm_joint_controller = g1_arm_joint_control.arm_joint_controller_node:main',
            'arm_pose_sender = g1_arm_joint_control.arm_pose_sender_node:main',
            'arm_sim_node = g1_arm_joint_control.arm_sim_node:main',
            'hand_target_ik = g1_arm_joint_control.hand_target_ik_node:main',
        ],
    },
)
