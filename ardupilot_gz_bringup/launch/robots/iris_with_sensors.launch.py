# Copyright 2026 ArduPilot.org.
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

"""Launch an iris quadcopter with xacro-generated sensors in Gazebo."""
from typing import List

import importlib.util
import os
import subprocess
import sys
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


_SENSOR_XACRO_MODELS = ("lidar", "camera", "depth_camera", "rgbd_camera")


def _load_bridge_generator():
    """Import bridge_generator.py sitting alongside this launch folder."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    module_path = os.path.join(pkg_project_bringup, "launch", "bridge_generator.py")
    spec = importlib.util.spec_from_file_location("bridge_generator", module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["bridge_generator"] = module
    spec.loader.exec_module(module)
    return module


def _xacro_to_sdf(xacro_path: str) -> str:
    """Run xacro synchronously, writing the SDF next to the xacro source."""
    sdf_path = xacro_path.replace(".xacro", ".sdf")
    subprocess.run(
        ["ros2", "run", "xacro", "xacro", "-o", sdf_path, xacro_path],
        check=True,
    )
    return sdf_path


def generate_robot_launch_actions(context: LaunchContext, *args, **kwargs):
    """Launch robot_state_publisher, bridge, SITL and spawn actions."""
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_description = get_package_share_directory("ardupilot_gz_description")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # 1) Run xacro on each sensor model so the sensor SDFs are up to date.
    #    The generated SDFs are written next to the xacro source so that
    #    ``model://<name>`` lookups through GZ_SIM_RESOURCE_PATH resolve
    #    to the freshly generated file.
    sensor_sdf_paths: List[str] = []
    for model_name in _SENSOR_XACRO_MODELS:
        xacro_file = os.path.join(
            pkg_project_description, "models", model_name, "model.xacro"
        )
        sensor_sdf_paths.append(_xacro_to_sdf(xacro_file))

    # 2) Auto-generate the bridge.yaml from the union of sensors found in
    #    the base iris model plus every generated sensor SDF. This keeps
    #    the bridge in sync with whatever sensors are currently declared
    #    in the xacro sources.
    base_sdf = os.path.join(
        pkg_ardupilot_gazebo, "models", "iris_with_standoffs", "model.sdf"
    )
    bridge_generator = _load_bridge_generator()
    bridge_tmp = tempfile.NamedTemporaryFile(
        delete=False, suffix="_iris_with_sensors_bridge.yaml"
    )
    bridge_tmp.close()
    bridge_generator.generate_bridge_yaml(
        [base_sdf] + sensor_sdf_paths, bridge_tmp.name
    )

    # 3) Load the iris_with_sensors model and patch sim_address.
    sdf_file = os.path.join(
        pkg_project_description, "models", "iris_with_sensors", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    sim_address = LaunchConfiguration("sim_address").perform(context)
    robot_desc = robot_desc.replace(
        "<fdm_addr>127.0.0.1</fdm_addr>",
        f"<fdm_addr>{sim_address}</fdm_addr>",
    )

    sdf_tmp = tempfile.NamedTemporaryFile(
        delete=False, suffix="_iris_with_sensors.sdf"
    )
    sdf_tmp.write(robot_desc.encode())
    sdf_tmp.close()

    # 4) Hand over to the shared robot launch file.
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
            "sdf_file": sdf_tmp.name,
            "bridge_config_file": bridge_tmp.name,
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
            default_value="warehouse",
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
    """Generate a launch description for an iris quadcopter with sensors."""

    launch_arguments = generate_launch_arguments()

    return LaunchDescription(
        launch_arguments + [OpaqueFunction(function=generate_robot_launch_actions)]
    )
