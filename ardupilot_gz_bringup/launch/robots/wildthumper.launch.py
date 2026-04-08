# Copyright 2023 ArduPilot.org.
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
Launch an wild thumper rover in Gazebo and Rviz.

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
          /share/ardupilot_sitl/config/default_params/rover-skid.parm,
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
from launch.actions import OpaqueFunction

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


def generate_robot_launch_actions(context: LaunchContext, *args, **kwargs):
    """Launch the robot_state_publisher and ros_gz bridge nodes."""
    pkg_ardupilot_sitl_models = get_package_share_directory("ardupilot_sitl_models")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # Load SDF file.
    sdf_file = os.path.join(
        pkg_ardupilot_sitl_models, "models", "wildthumper_with_lidar", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Substitute `models://` with `package://ardupilot_sitl_models/models/`
    # for sdformat_urdf plugin used by robot_state_publisher
    robot_desc = robot_desc.replace(
        "model://wildthumper", "package://ardupilot_sitl_models/models/wildthumper"
    )

    robot_desc = robot_desc.replace(
        "model://wildthumper_with_lidar",
        "package://ardupilot_sitl_models/models/wildthumper_with_lidar",
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

    bridge_config_file = os.path.join(
        pkg_project_bringup, "config", "wildthumper_bridge.yaml"
    )

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
            "command": "ardurover",
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

    return [robot]


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments."""
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
                    "rover.parm",
                )
                + ","
                + os.path.join(
                    pkg_ardupilot_sitl,
                    "config",
                    "default_params",
                    "rover-skid.parm",
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
            default_value="playpen",
            description="Name for the world instance.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="wildthumper",
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
            default_value="0.15",
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
    """Generate a launch description for a iris quadrotor"""

    launch_arguments = generate_launch_arguments()

    return LaunchDescription(
        launch_arguments + [OpaqueFunction(function=generate_robot_launch_actions)]
    )
