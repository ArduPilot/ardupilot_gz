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
Launch an iris quadcopter with ArduPilot and MAVProxy in Gazebo and Rviz.

ros2 launch ardupilot_dds_tests bringup_sitl_dds.launch.py
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

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    # Include component launch files.
    iris_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_gz_bringup"),
                        "launch",
                        "iris.launch.py",
                    ]
                ),
            ]
        )
    )

    bringup_sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_dds_tests"),
                        "launch",
                        "bringup_sitl_dds.launch.py",
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
                "dds.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    return LaunchDescription(
        [
            iris_gz,
            bringup_sitl_dds,
        ]
    )
