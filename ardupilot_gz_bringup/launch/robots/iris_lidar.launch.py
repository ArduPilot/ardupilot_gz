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
Launch an iris quadcopter with lidar in Gazebo and Rviz.

ros2 launch ardupilot_sitl sitl_dds_udp.launch.py
transport:=udp4
port:=2019
synthetic_clock:=True
wipe:=False
model:=json
speedup:=1
slave:=0
instance:=0
defaults:=$(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/gazebo-iris.parm,
          $(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/dds_udp.parm
sim_address:=127.0.0.1
master:=tcp:127.0.0.1:5760
sitl:=127.0.0.1:5501
"""
from typing import List

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler

from launch.conditions import IfCondition

from launch.event_handlers import OnProcessStart

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_launch_actions(context: LaunchContext, *args, **kwargs):
    """Launch the robot_state_publisher and ros_gz bridge nodes."""
    pkg_ardupilot_gz_description = get_package_share_directory(
        "ardupilot_gz_description"
    )
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # Load SDF file.
    sdf_file = os.path.join(
        pkg_ardupilot_gz_description, "models", "iris_with_lidar", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Substitute `models://` with `package://ardupilot_gazebo/models/`
    # for sdformat_urdf plugin used by robot_state_publisher
    robot_desc = robot_desc.replace(
        "model://iris_with_standoffs",
        "package://ardupilot_gazebo/models/iris_with_standoffs",
    )

    # Substitute `models://` with `package://ardupilot_gz_description/models/`
    # for sdformat_urdf plugin used by robot_state_publisher
    robot_desc = robot_desc.replace(
        "model://lidar_3d", "package://ardupilot_gz_description/models/lidar_3d"
    )

    # The robot description and ros_gz bridge config are chosen based
    # on the `lidar_dim` argument. The default is 3d.
    lidar_dim = LaunchConfiguration("lidar_dim").perform(context)
    config_name = "iris_3Dlidar_bridge.yaml"
    if lidar_dim == "3":
        log = LogInfo(msg="Using iris_with_3d_lidar_model")
    elif lidar_dim == "2":
        log = LogInfo(msg="Using iris_with_2d_lidar_model")
        robot_desc = robot_desc.replace("models/lidar_3d", "models/lidar_2d")
        config_name = "iris_2Dlidar_bridge.yaml"
    else:
        log = LogInfo(msg="ERROR: unknown lidar dimensions! Defaulting to 3d lidar")

    # Ensure the ArduPilot plugin and SITL have a consistent sim_address
    sim_address = LaunchConfiguration("sim_address").perform(context)
    robot_desc = robot_desc.replace(
        "<fdm_addr>127.0.0.1</fdm_addr>",
        f"<fdm_addr>{sim_address}</fdm_addr>",
    )

    # Publish /tf and /tf_static.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=LaunchConfiguration("robot_name"),
        arguments=[
            "-world",
            "",
            "-param",
            "",
            "-name",
            LaunchConfiguration("robot_name"),
            "-topic",
            "/robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        output="screen",
    )

    # Bridge
    config_template_file = os.path.join(
        pkg_project_bringup, "config", config_name
    )
    with open(config_template_file, "r") as infp:
        config = infp.read()

    world_name = LaunchConfiguration("world_name").perform(context)
    config = config.replace(
        "{{ world_name }}",
        f"{world_name}",
    )

    robot_name = LaunchConfiguration("robot_name").perform(context)
    config = config.replace(
        "{{ robot_name }}",
        f"{robot_name}",
    )

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    config_file = temp_file.name
    with open(config_file, "w") as outfp:
        outfp.write(config)

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": config_file,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Relay - use instead of transform when Gazebo is only publishing odom -> base_link
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    on_robot_state_publisher_start = RegisterEventHandler(
        OnProcessStart(target_action=robot_state_publisher, on_start=[spawn_robot])
    )

    on_bridge_start = RegisterEventHandler(
        OnProcessStart(target_action=bridge, on_start=[topic_tools_tf])
    )

    return [
        log,
        robot_state_publisher,
        bridge,
        on_robot_state_publisher_start,
        on_bridge_start,
    ]


def generate_launch_description():
    """Generate a launch description for a iris quadrotor"""
    launch_arguments = generate_launch_arguments()

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
            "model": LaunchConfiguration("model"),
            "defaults": LaunchConfiguration("defaults"),
            "synthetic_clock": LaunchConfiguration("synthetic_clock"),
        }.items(),
    )

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    robot_launch_actions = OpaqueFunction(function=generate_robot_launch_actions)

    ld = LaunchDescription(launch_arguments)
    ld.add_action(sitl_dds)
    ld.add_action(robot_launch_actions)

    return ld


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments"""
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    return [
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
                ),
            ),
            description="Set path to default params for the iris with DDS.",
        ),
        DeclareLaunchArgument(
            "synthetic_clock",
            default_value="True",
        ),
        # topic_tools_tf
        DeclareLaunchArgument(
            "use_gz_tf", default_value="true", description="Use Gazebo TF."
        ),
        # bridge, spawn_robot
        DeclareLaunchArgument(
            "world_name",
            default_value="maze",
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
        DeclareLaunchArgument(
            "lidar_dim",
            default_value="3",
            description="Whether to use a 2D or 3D lidar",
        ),
    ]
