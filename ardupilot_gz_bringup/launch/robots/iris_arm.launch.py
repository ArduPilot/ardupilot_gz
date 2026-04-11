# Copyright 2024 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Launch an iris quadcopter with so_arm_100 in Gazebo and Rviz.
"""
from typing import List

import math
import os
import tempfile

from copy import deepcopy
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler

from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# TODO: make this available as a library function in so_arm_100_bringup
def generate_bridge_config(
    template_file: str, world_name: str, model_name: str, prefix: str = ""
) -> str:
    """Generate a ros_gz_bridge config file from a template"""
    with open(template_file, "r") as file:
        config = file.read()

    config = config.replace(
        "{{ world_name }}",
        f"{world_name}",
    )

    config = config.replace(
        "{{ model_name }}",
        f"{model_name}",
    )

    config = config.replace(
        "{{ prefix }}",
        f"{prefix}",
    )

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    temp_file_name = temp_file.name

    with open(temp_file_name, "w") as temp_file:
        temp_file.write(config)

    return temp_file_name


def generate_robot_launch_actions(context: LaunchContext, *args, **kwargs):
    """Launch the robot_state_publisher and ros_gz bridge nodes."""
    pkg_project_description = get_package_share_directory("ardupilot_gz_description")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # Load SDF files.
    model_sdf_file = os.path.join(
        pkg_project_description, "models", "iris_with_arm", "model.sdf"
    )
    with open(model_sdf_file, "r") as file:
        model_sdf_doc = ET.fromstring(file.read())

    arm_sdf_file = os.path.join(
        pkg_project_description, "models", "so_arm_100", "model.sdf"
    )
    with open(arm_sdf_file, "r") as file:
        arm_sdf_doc = ET.fromstring(file.read())

    # add the ros2_control element back into the sdf
    arm_model = arm_sdf_doc.find("model")
    model = model_sdf_doc.find("model")
    if arm_model is not None and model is not None:
        ros2_control = arm_model.find("ros2_control")
        model.append(deepcopy(ros2_control))

    # Substitute `models://` with `package://ardupilot_gazebo/models/`
    # for sdformat_urdf plugin used by robot_state_publisher
    ET.indent(model_sdf_doc, space="  ")
    robot_desc = ET.tostring(model_sdf_doc).decode()
    robot_desc = robot_desc.replace(
        "model://iris_with_standoffs",
        "package://ardupilot_gazebo/models/iris_with_standoffs",
    )

    robot_desc = robot_desc.replace(
        "model://so_arm_100",
        "package://ardupilot_gz_description/models/so_arm_100",
    )

    # Ensure the ArduPilot plugin and SITL have a consistent sim_address
    sim_address = LaunchConfiguration("sim_address").perform(context)
    robot_desc = robot_desc.replace(
        "<fdm_addr>127.0.0.1</fdm_addr>",
        f"<fdm_addr>{sim_address}</fdm_addr>",
    )

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    sdf_file_modified = temp_file.name

    with open(sdf_file_modified, "w") as temp_file:
        temp_file.write(robot_desc)

    bridge_config_file = os.path.join(pkg_project_bringup, "config", "iris_bridge.yaml")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_project_bringup,
                        "launch",
                        "robots",
                        "robot.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "use_gz_tf": LaunchConfiguration("use_gz_tf"),
            "sdf_file": sdf_file_modified,
            "bridge_config_file": bridge_config_file,
            "command": "arducopter",
            "robot_name": LaunchConfiguration("robot_name"),
            "world_name": LaunchConfiguration("world_name"),
            "model": LaunchConfiguration("model"),
            "defaults": LaunchConfiguration("defaults"),
            "synthetic_clock": LaunchConfiguration("synthetic_clock"),
            "sim_address": LaunchConfiguration("sim_address"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "R": LaunchConfiguration("R"),
            "P": LaunchConfiguration("P"),
            "Y": LaunchConfiguration("Y"),
            "instance": LaunchConfiguration("instance"),
            "sysid": LaunchConfiguration("sysid"),
            "use_instance_dir": LaunchConfiguration("use_instance_dir"),
            "use_dds_agent": LaunchConfiguration("use_dds_agent"),
        }.items(),
    )

    # so-arm-100
    # so_arm_100_bringup/launch/gz.launch.py
    # dof:=5
    # prefix:="arm_"
    # model_name:="so_arm_100"

    ardupilot_gz_iris_arm_moveit_config_path = os.path.join(
        get_package_share_directory("ardupilot_gz_iris_arm_moveit_config")
    )

    so_arm_100_description_path = os.path.join(
        get_package_share_directory("so_arm_100_description")
    )

    so_arm_100_bringup_path = os.path.join(
        get_package_share_directory("so_arm_100_bringup")
    )

    so_arm_100_moveit_config_path = os.path.join(
        get_package_share_directory("so_arm_100_moveit_config")
    )

    dof = LaunchConfiguration("dof").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    use_topic = (
        LaunchConfiguration("use_topic_hardware_interface").perform(context) == "true"
    )

    namespace = LaunchConfiguration("namespace").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    if namespace:
        controller_manager_name = f"/{namespace}/controller_manager"
    else:
        controller_manager_name = "/controller_manager"

    print(f"dof:                      {dof}")
    print(f"prefix:                   {prefix}")
    print(f"world_name:               {world_name}")
    print(f"namespace:                {namespace}")
    print(f"robot_name:               {robot_name}")
    print(f"controller_manager_name:   {controller_manager_name}")

    # ros2_control configuration (as not using gz_ros2_control)
    ros2_control_file = os.path.join(
        ardupilot_gz_iris_arm_moveit_config_path,
        "config",
        f"ros2_controllers.yaml",
    )

    print(f"ros2_control_file: {ros2_control_file}")

    # controller manager (if not using gz_ros2_control)
    controller_manager = Node(
        condition=IfCondition(LaunchConfiguration("use_topic_hardware_interface")),
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            ros2_control_file,
            {"use_sim_time": True},
        ],
        remappings=[
            ("/robot_description", "robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # namespace=namespace,
        arguments=[
            "arm_controller",
            "--controller-manager",
            controller_manager_name,
            "--param-file",
            ros2_control_file,
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # namespace=namespace,
        arguments=[
            "gripper_controller",
            "--controller-manager",
            controller_manager_name,
            "--param-file",
            ros2_control_file,
        ],
    )

    bridge_template_file = os.path.join(
        pkg_project_bringup,
        "config",
        "so_arm_100_bridge.yaml",
    )

    # Generate bridge config from template
    bridge_config_file = generate_bridge_config(
        bridge_template_file, world_name, robot_name, prefix
    )

    # ros_gz_bridge and relay when using topic based control
    topic_based_control_bridge = Node(
        condition=IfCondition(LaunchConfiguration("use_topic_hardware_interface")),
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name=f"{prefix}ros_gz_bridge",
        parameters=[{"config_file": bridge_config_file}],
        output="screen",
    )

    command_relay = Node(
        condition=IfCondition(LaunchConfiguration("use_topic_hardware_interface")),
        package="so_arm_100_bringup",
        executable=f"so_arm_100_{dof}dof_cmd_relay",
        namespace=namespace,
        parameters=[{"prefix": prefix}],
        output="screen",
    )

    wait_spawn_done = Node(
        package="ardupilot_gz_utils",
        executable="wait_for_bool",
        namespace=namespace,
        parameters=[{"topic": "spawn_done"}],
        output="screen",
    )

    nodes = [
        robot,
        wait_spawn_done,
        topic_based_control_bridge,
        command_relay,
    ]

    if use_topic:
        # Topic-based control: start controller_manager,
        # then spawn controllers after it starts
        nodes += [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=wait_spawn_done,
                    on_start=[controller_manager],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=controller_manager,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
        ]
    else:
        # gz_ros2_control path: spawn controllers after robot is spawned
        nodes.append(
            joint_state_broadcaster_spawner,
        )

    # Common controller spawning chain
    nodes += [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[gripper_controller_spawner],
            )
        ),
    ]

    return nodes


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments"""
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    return [
        # ros-args
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace.",
        ),
        # sitl_dds
        DeclareLaunchArgument(
            "model",
            default_value="json",
            description="Set simulation model. Set default to 'json' for Gazebo.",
        ),
        DeclareLaunchArgument(
            "defaults",
            default_value=(
                os.path.join(
                    pkg_ardupilot_sitl,
                    "config",
                    "default_params",
                    "copter.parm",
                )
                + ","
                + os.path.join(
                    pkg_ardupilot_sitl,
                    "config",
                    "default_params",
                    "gazebo-iris.parm",
                )
                + ","
                + os.path.join(
                    pkg_ardupilot_sitl,
                    "config",
                    "default_params",
                    "dds_udp.parm",
                )
                + ","
                + os.path.join(
                    pkg_ardupilot_sitl,
                    "config",
                    "default_params",
                    "dds_use_ns.parm",
                )
            ),
            description="Set path to default params for the iris with DDS.",
        ),
        DeclareLaunchArgument(
            "synthetic_clock",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "sim_address",
            default_value="127.0.0.1",
        ),
        DeclareLaunchArgument(
            "instance",
            default_value="0",
            description="Set instance of SITL "
            "(adds 10*instance to all port numbers).",
        ),
        DeclareLaunchArgument(
            "sysid",
            default_value="",
            description="Set SYSID_THISMAV.",
        ),
        DeclareLaunchArgument(
            "use_instance_dir",
            default_value="False",
            description="If True create instance directories for the eeprom.bin.",
        ),
        DeclareLaunchArgument(
            "use_dds_agent",
            default_value="True",
            description="If True launch the micro-ros-agent.",
        ),
        # topic_tools_tf
        DeclareLaunchArgument(
            "use_gz_tf", default_value="true", description="Use Gazebo TF."
        ),
        # bridge, spawn_robot
        DeclareLaunchArgument(
            "world_name",
            default_value="runway",
            description="Name for the world instance.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="iris",
            description="Name for the model instance.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="The initial 'x' position (m).",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="The initial 'y' position (m).",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.2",
            description="The initial 'z' position (m).",
        ),
        DeclareLaunchArgument(
            "R",
            default_value="0.0",
            description="The initial roll angle (radians).",
        ),
        DeclareLaunchArgument(
            "P",
            default_value="0.0",
            description="The initial pitch angle (radians).",
        ),
        DeclareLaunchArgument(
            "Y",
            default_value=f"{math.radians(90)}",
            description="The initial yaw angle (radians).",
        ),
        # so-100-arm
        DeclareLaunchArgument(
            "dof", default_value="5", description="DOF configuration - either 5 or 7"
        ),
        DeclareLaunchArgument(
            "prefix", default_value="", description="Prefix of joint and link names"
        ),
        DeclareLaunchArgument(
            "use_topic_hardware_interface",
            default_value="false",
            description="Use topic based hardware interface instead of gz_ros_control",
        ),
    ]


def generate_launch_description():
    """Generate a launch description for a iris quadcopter with robotic arm."""

    launch_arguments = generate_launch_arguments()

    return LaunchDescription(
        launch_arguments + [OpaqueFunction(function=generate_robot_launch_actions)]
    )
