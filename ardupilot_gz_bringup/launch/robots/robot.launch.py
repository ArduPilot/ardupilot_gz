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
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def replace_robot_name(input_file: str, robot_name: str, world_name: str) -> str:
    with open(input_file, "r") as file:
        config = file.read()

    config = config.replace(
        "{{ world_name }}",
        f"{world_name}",
    )

    config = config.replace(
        "{{ robot_name }}",
        f"{robot_name}",
    )

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    temp_file_name = temp_file.name

    with open(temp_file_name, "w") as temp_file:
        temp_file.write(config)

    return temp_file_name


def launch_spawn_robot(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    """Return a Gazebo spawn robot launch description"""
    # Get substitutions for arguments
    namespace = LaunchConfiguration("namespace")
    name = LaunchConfiguration("robot_name")
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
        namespace=namespace,
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

    publish_spawn_done = Node(
        package="ardupilot_gz_utils",
        executable="send_bool",
        namespace=namespace,
        parameters=[{"topic": "spawn_done"}],
        output="screen",
    )

    delayed_publish_spawn_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[publish_spawn_done],
        ),
    )

    return [
        spawn_robot,
        delayed_publish_spawn_done,
    ]


def launch_state_pub_with_bridge(
    context: LaunchContext,
) -> List[LaunchDescriptionEntity]:
    namespace = LaunchConfiguration("namespace").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
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
        namespace=namespace,
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
    tmp_bridge_file = replace_robot_name(bridge_config_file, robot_name, world_name)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
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
        namespace=namespace,
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
    name = LaunchConfiguration("robot_name").perform(context)
    command = LaunchConfiguration("command").perform(context)
    instance = int(LaunchConfiguration("instance").perform(context))

    # Optional arguments.
    sysid = LaunchConfiguration("sysid").perform(context)
    if not sysid:
        sysid = instance + 1

    # Ports
    port_offset = 10 * instance
    dds_port = 2019 + port_offset
    master_port = 5760 + port_offset
    sitl_port = 5501 + port_offset
    mavlink_out = 14550 + port_offset
    sim_address = LaunchConfiguration("sim_address").perform(context)

    # ardupilot_sitl
    sitl_arguments = {
        "command": command,
        "wipe": "False",
        "speedup": "1",
        "slave": "0",
        "instance": f"{instance}",
        "sysid": f"{sysid}",
        "model": LaunchConfiguration("model"),
        "defaults": LaunchConfiguration("defaults"),
        "synthetic_clock": LaunchConfiguration("synthetic_clock"),
        "sim_address": sim_address,
        "master": f"tcp:{sim_address}:{master_port}",
        "sitl": f"{sim_address}:{sitl_port}",
        "out": f"{sim_address}:{mavlink_out}",
        "use_instance_dir": LaunchConfiguration("use_instance_dir"),
    }

    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_ardupilot_sitl,
                        "launch",
                        "sitl_mavproxy.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments=sitl_arguments.items(),
        condition=UnlessCondition(LaunchConfiguration("use_dds_agent")),
    )

    # ardupilot_sitl + micro_ros_agent
    sitl_dds_arguments = sitl_arguments.copy()
    sitl_dds_arguments.update(
        {
            "micro_ros_agent_ns": name,
            "transport": "udp4",
            "port": f"{dds_port}",
        }
    )

    # Include component launch files.
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_ardupilot_sitl,
                        "launch",
                        "sitl_dds_udp.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments=sitl_dds_arguments.items(),
        condition=IfCondition(LaunchConfiguration("use_dds_agent")),
    )

    return [sitl, sitl_dds]


def generate_launch_arguments() -> List[LaunchDescriptionEntity]:
    """Generate a list of launch arguments"""
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
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
                    pkg_ardupilot_gazebo,
                    "config",
                    "gazebo-iris-gimbal.parm",
                )
                + ","
                + os.path.join(
                    pkg_ardupilot_sitl,
                    "config",
                    "default_params",
                    "dds_udp.parm",
                ),
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
        DeclareLaunchArgument(
            "sdf_file",
            default_value=os.path.join(
                pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"
            ),
            description="Path to robot SDF file.",
        ),
        DeclareLaunchArgument(
            "bridge_config_file",
            default_value=os.path.join(
                pkg_project_bringup, "config", "iris_bridge.yaml"
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
            default_value="0.0",
            description="The initial yaw angle (radians).",
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
