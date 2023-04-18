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
Launch an iris quadcopter in Gazebo and Rviz.

ros2 launch ardupilot_sitl sitl_dds.launch.py
tty0:=./dev/ttyROS0
tty1:=./dev/ttyROS1
refs:=$(ros2 pkg prefix ardupilot_sitl)
      /share/ardupilot_sitl/config/dds_xrce_profile.xml
baudrate:=115200 device:=./dev/ttyROS0
synthetic_clock:=True
wipe:=True
model:=json
speedup:=1
slave:=0
instance:=0
uartC:=uart:./dev/ttyROS1
defaults:=$(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/gazebo-iris.parm,
          $(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/dds.parm
sim_address:=127.0.0.1
master:=tcp:127.0.0.1:5760
sitl:=127.0.0.1:5501
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # Include component launch files.
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_dds.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "tty0": "./dev/ttyROS0",
            "tty1": "./dev/ttyROS1",
            "refs": PathJoinSubstitution(
                [
                    FindPackageShare("ardupilot_sitl"),
                    "config",
                    "dds_xrce_profile.xml",
                ]
            ),
            "baudrate": "115200",
            "device": "./dev/ttyROS0",
            "synthetic_clock": "True",
            "wipe": "True",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "uartC": "uart:./dev/ttyROS1",
            "defaults": os.path.join(
                pkg_ardupilot_gazebo,
                "config",
                "gazebo-iris-gimbal.parm",
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    # Robot description.

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Load SDF file.
    sdf_file = os.path.join(
        pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

    # Remap the /tf and /tf_static under /ignore as the TF conflicts.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": "iris/"},
        ],
        remappings=[
            ("/tf", "/ignore/tf"),
            ("/tf_static", "/ignore/tf_static"),
        ],
    )

    # Bridge.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "iris_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            sitl_dds,
            robot_state_publisher,
            bridge,
        ]
    )
