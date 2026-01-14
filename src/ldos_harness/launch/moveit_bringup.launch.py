"""
moveit_bringup.launch.py - Launch MoveIt 2 for Panda robot

This launch file starts:
1. MoveIt move_group node
2. Optional RViz with MoveIt plugin
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a yaml file from a package with error logging."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"WARNING: Config file not found: {absolute_file_path}")
        return None
    except yaml.YAMLError as e:
        print(f"WARNING: YAML parse error in {absolute_file_path}: {e}")
        return None
    except Exception as e:
        print(f"WARNING: Failed to load {absolute_file_path}: {e}")
        return None


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    pkg_ldos_harness = get_package_share_directory("ldos_harness")

    # Evaluate use_sim_time for use in parameters
    use_sim_time_bool = use_sim_time.perform(context).lower() == "true"

    # Robot description (must match sim_bringup)
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [pkg_ldos_harness, "config", "panda_gz.urdf.xacro"]
                ),
                " ",
                "use_sim_time:=",
                use_sim_time,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # SRDF for MoveIt
    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [pkg_ldos_harness, "config", "panda.srdf.xacro"]
                ),
            ]
        ),
        value_type=str,
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # MoveIt configuration (loaded from YAML to ensure proper string_array typing)
    # This includes planning_pipelines, kinematics, controllers, trajectory execution, etc.
    moveit_config_yaml = load_yaml("ldos_harness", "config/moveit.yaml")
    if moveit_config_yaml is None:
        # Fallback config - should not happen if package is built correctly
        moveit_config_yaml = {
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
        }

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config_yaml,
            {"use_sim_time": use_sim_time_bool},
        ],
    )

    # RViz (optional)
    rviz_config_file = PathJoinSubstitution(
        [pkg_ldos_harness, "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config_yaml,
            {"use_sim_time": use_sim_time_bool},
        ],
        condition=IfCondition(use_rviz),
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz for visualization",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
