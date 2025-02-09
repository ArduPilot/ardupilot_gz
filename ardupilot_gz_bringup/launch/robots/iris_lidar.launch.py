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
Launch an iris quadcopter in Gazebo and Rviz.

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
import tempfile
from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def choose_lidar(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    # Robot description and ros_gz bridge config chosen based on passed lidar_dimension argument
    lidar_dim = LaunchConfiguration("lidar_dim").perform(context)
    name = LaunchConfiguration("name")
    pkg_ardupilot_gz_description = get_package_share_directory(
        "ardupilot_gz_description"
    )
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    sdf_file = str(
        Path(pkg_ardupilot_gz_description) / "models" / "iris_with_lidar" / "model.sdf"
    )

    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    ros_gz_bridge_config = "iris_3Dlidar_bridge.yaml"

    # Load SDF file and choose ros_gz bridge config based on lidar dimensions
    if lidar_dim == "2":
        log = LogInfo(msg="Using iris_with_2d_lidar_model ")
        robot_desc = robot_desc.replace(
            "<uri>model://lidar_2d</uri>", "<uri>model://lidar_2d</uri>"
        )
        ros_gz_bridge_config = "iris_2Dlidar_bridge.yaml"
    elif lidar_dim == "3":
        log = LogInfo(msg="Using iris_with_3d_lidar_model")
        robot_desc = robot_desc.replace(
            "<uri>model://lidar_2d</uri>", "<uri>model://lidar_3d</uri>"
        )
        ros_gz_bridge_config = "iris_3Dlidar_bridge.yaml"
    else:
        log = LogInfo(msg="ERROR: unknown lidar dimensions! Defaulting to 3d lidar")
        robot_desc = robot_desc.replace(
            "<uri>model://lidar_2d</uri>", "<uri>model://lidar_3d</uri>"
        )
        ros_gz_bridge_config = "iris_3Dlidar_bridge.yaml"

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    sdf_file_modified = temp_file.name

    with open(sdf_file_modified, "w") as temp_file:
        temp_file.write(robot_desc)

    sitl_config_file = str(
        Path(pkg_ardupilot_sitl) / "config" / "default_params" / "gazebo-iris.parm"
    )

    bridge_config_file = str(
        Path(pkg_project_bringup) / "config" / ros_gz_bridge_config
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
            "command": "arducopter",
            "name": name,
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

    return [log, robot]


def generate_launch_arguments() -> List[LaunchDescriptionEntity]:
    """Generate a list of launch arguments"""
    return [
        DeclareLaunchArgument(
            "use_gz_tf", default_value="true", description="Use Gazebo TF."
        ),
        DeclareLaunchArgument(
            "lidar_dim",
            default_value="3",
            description="Whether to use a 2D or 3D lidar",
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
    """Generate a launch description for a iris quadrotor"""

    launch_arguments = generate_launch_arguments()

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=choose_lidar)])
