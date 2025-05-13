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
import tempfile
from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_arguments() -> List[LaunchDescriptionEntity]:
    """Generate a list of launch arguments"""
    return [
        DeclareLaunchArgument(
            "use_gz_tf", default_value="true", description="Use Gazebo TF."
        ),
        # Gazebo model launch arguments.
        DeclareLaunchArgument(
            "name",
            default_value="wildthumper",
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
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for a wild thumper rover."""

    launch_arguments = generate_launch_arguments()

    pkg_ardupilot_sitl_models = get_package_share_directory("ardupilot_sitl_models")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    # Load SDF file.
    sdf_file = str(
        Path(pkg_ardupilot_sitl_models)
        / "models"
        / "wildthumper_with_lidar"
        / "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

        # substitute `models://` with `package://ardupilot_sitl_models/models/`
        # for sdformat_urdf plugin used by robot_state_publisher
        robot_desc = robot_desc.replace(
            "model://wildthumper", "package://ardupilot_sitl_models/models/wildthumper"
        )

        robot_desc = robot_desc.replace(
            "model://wildthumper_with_lidar",
            "package://ardupilot_sitl_models/models/wildthumper_with_lidar",
        )

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    sdf_file_modified = temp_file.name

    with open(sdf_file_modified, "w") as temp_file:
        temp_file.write(robot_desc)

    sitl_config_file = str(
        Path(pkg_ardupilot_sitl) / "config" / "default_params" / "rover-skid.parm"
    )

    bridge_config_file = str(
        Path(pkg_project_bringup) / "config" / "wildthumper_bridge.yaml"
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_gz_bringup"),
                        "launch",
                        "robots",
                        "robot.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "use_gz_tf": LaunchConfiguration("use_gz_tf"),
            "sdf_file": sdf_file_modified,
            "sitl_config_file": sitl_config_file,
            "bridge_config_file": bridge_config_file,
            "command": "ardurover",
            "name": LaunchConfiguration("name"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "R": LaunchConfiguration("R"),
            "P": LaunchConfiguration("P"),
            "Y": LaunchConfiguration("Y"),
            "instance": LaunchConfiguration("instance"),
            "sysid": LaunchConfiguration("sysid"),
        }.items(),
    )

    return LaunchDescription(
        launch_arguments
        + [
            robot,
        ]
    )
