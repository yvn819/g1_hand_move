import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_controller_params = os.path.join(
        get_package_share_directory("g1_arm_joint_control"),
        "config",
        "arm_control.yaml",
    )
    default_sim_params = os.path.join(
        get_package_share_directory("g1_arm_joint_control"),
        "config",
        "arm_sim.yaml",
    )

    controller_params_file = LaunchConfiguration("controller_params_file")
    sim_params_file = LaunchConfiguration("sim_params_file")
    conda_python = LaunchConfiguration("conda_python")
    use_conda_prefix = LaunchConfiguration("use_conda_prefix")

    args = [
        DeclareLaunchArgument(
            "controller_params_file",
            default_value=default_controller_params,
            description="Path to arm controller parameter file",
        ),
        DeclareLaunchArgument(
            "sim_params_file",
            default_value=default_sim_params,
            description="Path to MuJoCo sim parameter file",
        ),
        DeclareLaunchArgument(
            "conda_python",
            default_value="/home/yutong/miniconda3/envs/g1_control/bin/python",
            description="Python interpreter used to run arm_sim_node",
        ),
        DeclareLaunchArgument(
            "use_conda_prefix",
            default_value="true",
            description="Run arm_sim_node with conda_python prefix",
        ),
    ]

    sim_with_prefix = Node(
        package="g1_arm_joint_control",
        executable="arm_sim_node",
        name="arm_sim_node",
        output="screen",
        prefix=[conda_python],
        parameters=[sim_params_file],
        condition=IfCondition(use_conda_prefix),
    )

    sim_no_prefix = Node(
        package="g1_arm_joint_control",
        executable="arm_sim_node",
        name="arm_sim_node",
        output="screen",
        parameters=[sim_params_file],
        condition=UnlessCondition(use_conda_prefix),
    )

    arm_controller = Node(
        package="g1_arm_joint_control",
        executable="arm_joint_controller",
        name="arm_joint_controller",
        output="screen",
        parameters=[controller_params_file],
    )

    return LaunchDescription(args + [sim_with_prefix, sim_no_prefix, arm_controller])
