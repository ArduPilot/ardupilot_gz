# Copyright 2025 ArduPilot.org.
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

import os
import tempfile
from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def replace_robot_name(input_file: str, robot_name: str) -> str:
    with open(input_file, "r") as file:
        content = file.read()

    replaced_content = content.replace("<robot_name>", robot_name)

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    temp_file_name = temp_file.name

    with open(temp_file_name, "w") as temp_file:
        temp_file.write(replaced_content)

    return temp_file_name


def launch_spawn_robot(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    """Return a Gazebo spawn robot launch description"""
    # Get substitutions for arguments
    name = LaunchConfiguration("name")
    pos_x = LaunchConfiguration("x")
    pos_y = LaunchConfiguration("y")
    pos_z = LaunchConfiguration("z")
    rot_r = LaunchConfiguration("R")
    rot_p = LaunchConfiguration("P")
    rot_y = LaunchConfiguration("Y")

    # spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=name,
        arguments=[
            "-world",
            "",
            "-param",
            "",
            "-name",
            name,
            "-topic",
            "robot_description",
            "-x",
            pos_x,
            "-y",
            pos_y,
            "-z",
            pos_z,
            "-R",
            rot_r,
            "-P",
            rot_p,
            "-Y",
            rot_y,
        ],
        output="screen",
    )
    return [spawn_robot]


def launch_state_pub_with_bridge(
    context: LaunchContext,
) -> List[LaunchDescriptionEntity]:
    name = LaunchConfiguration("name").perform(context)
    sdf_file = LaunchConfiguration("sdf_file").perform(context)
    bridge_config_file = LaunchConfiguration("bridge_config_file").perform(context)
    instance = int(LaunchConfiguration("instance").perform(context))

    # Compute ports
    port_offset = 10 * instance
    control_port = 9002 + port_offset

    # Load SDF file.
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    robot_desc = robot_desc.replace(
        "<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{control_port}</fdm_port_in>"
    )

    # Publish /tf and /tf_static.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=name,
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # Bridge.
    tmp_bridge_file = replace_robot_name(bridge_config_file, name)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=name,
        parameters=[
            {
                "config_file": tmp_bridge_file,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Relay - use instead of transform when Gazebo is only publishing odom -> base_link
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        namespace=name,
        arguments=[
            "gz/tf",
            "tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    event = RegisterEventHandler(
        OnProcessStart(target_action=bridge, on_start=[topic_tools_tf])
    )

    return [robot_state_publisher, bridge, event]


def launch_sitl_dds(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    # Required arguments.
    name = LaunchConfiguration("name").perform(context)
    command = LaunchConfiguration("command").perform(context)
    sitl_config_file = LaunchConfiguration("sitl_config_file").perform(context)
    instance = int(LaunchConfiguration("instance").perform(context))

    # Optional arguments.
    sysid = LaunchConfiguration("sysid").perform(context)
    if not sysid:
        sysid = instance + 1

    # Ports
    port_offset = 10 * instance
    master_port = 5760 + port_offset
    sitl_port = 5501 + port_offset
    sim_address = "127.0.0.1"

    tty0 = f"./dev/ttyROS{instance * 10}"
    tty1 = f"./dev/ttyROS{instance * 10 + 1}"

    # Include component launch files.
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_dds_udp.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            # virtual_ports
            "tty0": tty0,
            "tty1": tty1,
            # micro_ros_agent
            "micro_ros_agent_ns": name,
            "baudrate": "115200",
            "device": tty0,
            # ardupilot_sitl
            "command": command,
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": f"{instance}",
            "sysid": f"{sysid}",
            "uartC": f"uart:{tty1}",
            "defaults": sitl_config_file
            + ","
            + str(
                Path(pkg_ardupilot_sitl)
                / "config"
                / "default_params"
                / "gazebo-iris.parm"
            ),
            "sim_address": sim_address,
            "master": f"tcp:{sim_address}:{master_port}",
            "sitl": f"{sim_address}:{sitl_port}",
        }.items(),
    )

    return [sitl_dds]


def generate_launch_arguments() -> List[LaunchDescriptionEntity]:
    """Generate a list of launch arguments"""
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    return [
        DeclareLaunchArgument(
            "use_gz_tf", default_value="true", description="Use Gazebo TF."
        ),
        DeclareLaunchArgument(
            "sdf_file",
            default_value=str(
                Path(pkg_ardupilot_gazebo) / "models" / "iris_with_gimbal" / "model.sdf"
            ),
            description="Path to robot SDF file.",
        ),
        DeclareLaunchArgument(
            "sitl_config_file",
            default_value=str(
                Path(pkg_ardupilot_gazebo) / "config" / "gazebo-iris-gimbal.parm"
            ),
            description="Path to SITL robot config file.",
        ),
        DeclareLaunchArgument(
            "bridge_config_file",
            default_value=str(
                Path(pkg_project_bringup) / "config" / "iris_bridge.yaml"
            ),
            description="Path to ROS Gazebo Bridge config file.",
        ),
        DeclareLaunchArgument(
            "command",
            default_value="arducopter",
            description="Type of vehicle to.",
            choices=[
                "antennatracker",
                "arducopter-heli",
                "ardurover",
                "blimp",
                "arducopter",
                "arduplane",
                "ardusub",
            ],
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
        # Gazebo model launch arguments.
        DeclareLaunchArgument(
            "name",
            default_value="iris",
            description="Name for the model instance.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0",
            description="The intial 'x' position (m).",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0",
            description="The intial 'y' position (m).",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0",
            description="The intial 'z' position (m).",
        ),
        DeclareLaunchArgument(
            "R",
            default_value="0",
            description="The intial roll angle (radians).",
        ),
        DeclareLaunchArgument(
            "P",
            default_value="0",
            description="The intial pitch angle (radians).",
        ),
        DeclareLaunchArgument(
            "Y",
            default_value="0",
            description="The intial yaw angle (radians).",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for a iris quadcopter."""

    launch_arguments = generate_launch_arguments()

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    opfunc_robot_state_publisher = OpaqueFunction(function=launch_state_pub_with_bridge)
    opfunc_spawn_robot = OpaqueFunction(function=launch_spawn_robot)
    opfunc_sitl = OpaqueFunction(function=launch_sitl_dds)
    ld = LaunchDescription(launch_arguments)
    ld.add_action(opfunc_robot_state_publisher)
    ld.add_action(opfunc_sitl)
    ld.add_action(opfunc_spawn_robot)

    return ld
