from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_controller_params = os.path.join(
        get_package_share_directory('g1_arm_joint_control'),
        'config',
        'arm_control.yaml',
    )
    default_sim_params = os.path.join(
        get_package_share_directory('g1_arm_joint_control'),
        'config',
        'arm_sim.yaml',
    )
    default_ik_params = os.path.join(
        get_package_share_directory('g1_arm_joint_control'),
        'config',
        'hand_target_ik.yaml',
    )

    params_arg = DeclareLaunchArgument(
        'controller_params_file',
        default_value=default_controller_params,
        description='Path to parameter file for arm joint controller',
    )
    sim_params_arg = DeclareLaunchArgument(
        'sim_params_file',
        default_value=default_sim_params,
        description='Path to parameter file for MuJoCo arm simulator',
    )
    ik_params_arg = DeclareLaunchArgument(
        'ik_params_file',
        default_value=default_ik_params,
        description='Path to parameter file for hand target IK node',
    )

    python_env = DeclareLaunchArgument(
            "conda_python",
            default_value="/home/yutong/miniconda3/envs/g1_control/bin/python",
            description="Python interpreter inside conda environment"
    )

    sim_node = Node(
        package='g1_arm_joint_control',
        executable='arm_sim_node',
        name='arm_sim_node',
        output='screen',
        prefix=[LaunchConfiguration('conda_python')],
        parameters=[LaunchConfiguration('sim_params_file')],
    )

    controller = Node(
        package='g1_arm_joint_control',
        executable='arm_joint_controller',
        name='arm_joint_controller',
        output='screen',
        parameters=[LaunchConfiguration('controller_params_file')],
    )

    hand_target_ik = Node(
        package='g1_arm_joint_control',
        executable='hand_target_ik',
        name='hand_target_ik',
        output='screen',
        prefix=[LaunchConfiguration('conda_python')],
        parameters=[LaunchConfiguration('ik_params_file')],
    )

    return LaunchDescription(
        [params_arg, sim_params_arg, ik_params_arg, python_env, sim_node, controller, hand_target_ik]
    )
